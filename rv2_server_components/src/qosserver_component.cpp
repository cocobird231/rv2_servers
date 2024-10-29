#include <rv2_server_components/qosserver_component.h>

namespace rv2_interfaces
{

QoSServer::QoSServer(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(DEFAULT_QOS_SERVER_NODENAME, options), 
    mQID_(0)
{
    // Get parameters
    GetParamRawPtr(this, "qosService", mServiceName_, mServiceName_, "qosService: ", false);
    GetParamRawPtr(this, "qosServerDirPath", mQoSServerDirPath_, mQoSServerDirPath_, "qosServerDirPath: ", false);
    if (mServiceName_ == "")
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer] QoS service name is empty, set to default.");
        mServiceName_ = DEFAULT_QOS_SRV_NAME;
    }
    if (mQoSServerDirPath_ == "")
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer] QoS server directory path is empty, set to default.");
        mQoSServerDirPath_ = DEFAULT_QOS_SERVER_DIR_PATH;
    }

    mQoSFilePath_ = (fs::path)mQoSServerDirPath_ / "qos" ;
    mISrvFilePath_ = (fs::path)mQoSServerDirPath_ / "isrv";
    fs::create_directories(mQoSFilePath_);
    fs::create_directories(mISrvFilePath_);

    this->_loadISrvFromJSON();
    {
        // Print map
        std::lock_guard<std::mutex> topicDevInfoStatusMapLock(mTopicDevInfoStatusMapMtx_);
        for (const auto& [nodeName, tPair] : mTopicDevInfoStatusMap_)
        {
            const auto& [tInfo, status] = tPair;
            printf("%s [%s] (%s)\n", nodeName.c_str(), GetQosTypeStr(tInfo.qos_type).c_str(), status ? "registered" : "unregistered");
        }
    }

    {// Load QoS profiles from the file.
        if (this->loadQmapFromJSON(true))
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer] Load QoS profiles from %s.", mQoSFilePath_.generic_string().c_str());
            auto tmp = SafeLoad(&mTopicQoSMap_, mTopicQoSMapMtx_);
            printf("Qmap found, size: %ld, qid: %ld\n", tmp.size(), mQID_.load());
            for (const auto& [topicName, topicQoS] : tmp)
            {
                if (topicQoS.isPubQoSValid())
                {
                    const auto& q = topicQoS[msg::TopicQosProfile::QOS_TYPE_PUBLISHER];
                    printf("%s [publisher] (%02d/%02ld/%02d)\n", topicName.c_str(), q.history, q.depth, q.reliability);
                }
                if (topicQoS.isSubQoSValid())
                {
                    const auto& q = topicQoS[msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION];
                    printf("%s [subscription] (%02d/%02ld/%02d)\n", topicName.c_str(), q.history, q.depth, q.reliability);
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer] Load QoS profiles failed: %s.", mQoSFilePath_.generic_string().c_str());
        }
    }

    // QoS registration and request services.
    mQoSRegSrv_ = this->create_service<srv::TopicQosProfileReg>(QoSServer_TopicQosProfileRegSrvName(mServiceName_), std::bind(&QoSServer::_qosRegSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    mQoSReqSrv_ = this->create_service<srv::TopicQosProfileReq>(QoSServer_TopicQosProfileReqSrvName(mServiceName_), std::bind(&QoSServer::_qosReqSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    // Topic device information registration service.
    mTopicDevInfoRegSrv_ = this->create_service<srv::TopicDeviceInfoReg>(QoSServer_TopicDeviceInfoRegSrvName(mServiceName_), std::bind(&QoSServer::_topicDevInfoRegSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    // QoS server service.
    mQoSSrv_ = this->create_service<srv::QosServer>(QoSServer_QosServerSrvName(mServiceName_), std::bind(&QoSServer::_qosSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "[QoSServer] Constructed.");
}



void QoSServer::_qosRegSrvCb(const std::shared_ptr<srv::TopicQosProfileReg::Request> request, 
                            std::shared_ptr<srv::TopicQosProfileReg::Response> response)
{
    std::lock_guard<std::mutex> qosRegSrvLock(mQoSRegSrvMtx_);
    const auto& tName = request->topic_qos_profile.topic_name;
    const auto& tType = request->topic_qos_profile.qos_type;
    const auto& tQoS = request->topic_qos_profile.qos_profile;

    // Check invalid request.
    if (tName == "" && !request->save_qmap && !request->clear_profiles)
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegSrvCb] Request: %s [%s] failed: invalid request.", 
            tName.c_str(), 
            GetQosTypeStr(tType).c_str());
        response->response = ServiceResponseStatus::SRV_RES_ERROR;
        response->reason = TopicQoSExceptionMsg[TopicQoSException::QOS_EXC_INVALID_TOPIC_NAME];
        return;
    }

    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;

    if (request->save_qmap)// Save temporary topic QoS profile.
    {
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegSrvCb] Save qmap request.");
        auto result = this->setTopicQoSMap();
        response->response = result.result ? ServiceResponseStatus::SRV_RES_SUCCESS : ServiceResponseStatus::SRV_RES_ERROR;
        if (response->response != ServiceResponseStatus::SRV_RES_SUCCESS)
        {
            response->reason = result.reason;
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegSrvCb] Save qmap failed.");
        }
    }
    else if (request->clear_profiles)// Recover whole temporary QoS profiles.
    {
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegSrvCb] Recover tmp qmap.");
        this->recoverTmpQoSProfile();
    }
    else if (request->remove_profile)// Remove single temporary QoS profile.
    {
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegSrvCb] Request: remove %s.", 
            tName.c_str());
        this->removeTmpQoSProfile(tName.c_str());
    }
    else// Set temporary QoS profile.
    {
        // Check invalid QoS type.
        if (tType != msg::TopicQosProfile::QOS_TYPE_PUBLISHER && 
            tType != msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION && 
            tType != msg::TopicQosProfile::QOS_TYPE_BOTH)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegSrvCb] Request: set %s [%s] failed: invalid qos_type.", 
                tName.c_str(), 
                GetQosTypeStr(tType).c_str());
            response->response = ServiceResponseStatus::SRV_RES_ERROR;
            response->reason = TopicQoSExceptionMsg[TopicQoSException::QOS_EXC_INVALID_PROPERTY];
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegSrvCb] Request: set %s (%-2d/%-2ld/%-2d) [%s].", 
            tName.c_str(), tQoS.history, tQoS.depth, tQoS.reliability, GetQosTypeStr(tType).c_str());
        response->response = this->setTmpQoSProfile(request->topic_qos_profile, false) ? ServiceResponseStatus::SRV_RES_SUCCESS : ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;// Set temporary QoS profile without checking.
        if (response->response != ServiceResponseStatus::SRV_RES_SUCCESS)
        {
            response->reason = "[QoSServer::_qosRegSrvCb] Unexpected error.";
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegSrvCb] Request: set %s [%s] failed.", tName.c_str(), GetQosTypeStr(tType).c_str());
        }
    }
    return;
}



void QoSServer::_qosReqSrvCb(const std::shared_ptr<srv::TopicQosProfileReq::Request> request, 
                            std::shared_ptr<srv::TopicQosProfileReq::Response> response)
{
    std::lock_guard<std::mutex> qosReqSrvLock(mQoSReqSrvMtx_);
    // Check invalid request.
    if (request->topic_name == "")
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosReqSrvCb] Request: %s [%s] failed: invalid request.", request->topic_name.c_str(), GetQosTypeStr(request->qos_type).c_str());
        response->response = ServiceResponseStatus::SRV_RES_ERROR;
        response->reason = TopicQoSExceptionMsg[TopicQoSException::QOS_EXC_INVALID_TOPIC_NAME];
        return;
    }

    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;

    // Safe copy of mTopicQoSMap_.
    auto topicQoSMapCopy = SafeLoad(&mTopicQoSMap_, mTopicQoSMapMtx_);
    try
    {
        if (request->topic_name == "all")
        {
            for (const auto& [topicName, tQoS] : topicQoSMapCopy)
            {
                if (tQoS.isPubQoSValid() && (request->qos_type == msg::TopicQosProfile::QOS_TYPE_PUBLISHER || request->qos_type == msg::TopicQosProfile::QOS_TYPE_BOTH))
                {
                    msg::TopicQosProfile tQoSMsg;
                    tQoSMsg.topic_name = topicName;
                    tQoSMsg.qos_type = msg::TopicQosProfile::QOS_TYPE_PUBLISHER;
                    tQoSMsg.qos_profile = CvtRmwQoSToMsg(tQoS[msg::TopicQosProfile::QOS_TYPE_PUBLISHER]);
                    response->topic_qos_profile_vec.push_back(tQoSMsg);
                }
                if (tQoS.isSubQoSValid() && (request->qos_type == msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION || request->qos_type == msg::TopicQosProfile::QOS_TYPE_BOTH))
                {
                    msg::TopicQosProfile tQoSMsg;
                    tQoSMsg.topic_name = topicName;
                    tQoSMsg.qos_type = msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION;
                    tQoSMsg.qos_profile = CvtRmwQoSToMsg(tQoS[msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION]);
                    response->topic_qos_profile_vec.push_back(tQoSMsg);
                }
            }
        }
        else
        {
            // Check invalid topic type.
            if (request->qos_type != msg::TopicQosProfile::QOS_TYPE_PUBLISHER && 
                request->qos_type != msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION && 
                request->qos_type != msg::TopicQosProfile::QOS_TYPE_BOTH)
            {
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosReqSrvCb] Request: %s [%s] failed: invalid qos_type.", 
                    request->topic_name.c_str(), 
                    GetQosTypeStr(request->qos_type).c_str());
                response->response = ServiceResponseStatus::SRV_RES_ERROR;
                response->reason = TopicQoSExceptionMsg[TopicQoSException::QOS_EXC_INVALID_QOS_TYPE];
                return;
            }

            if (topicQoSMapCopy.find(request->topic_name) == topicQoSMapCopy.end())
                throw TopicQoSException::QOS_EXC_INVALID_TOPIC_NAME;

            TopicQoS qos = topicQoSMapCopy[request->topic_name];
            if (request->qos_type == msg::TopicQosProfile::QOS_TYPE_PUBLISHER || request->qos_type == msg::TopicQosProfile::QOS_TYPE_BOTH)
            {
                msg::TopicQosProfile tQoSMsg;
                tQoSMsg.topic_name = request->topic_name;
                tQoSMsg.qos_type = msg::TopicQosProfile::QOS_TYPE_PUBLISHER;
                tQoSMsg.qos_profile = CvtRmwQoSToMsg(qos[msg::TopicQosProfile::QOS_TYPE_PUBLISHER]);
                response->topic_qos_profile_vec.push_back(tQoSMsg);
            }
            if (request->qos_type == msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION || request->qos_type == msg::TopicQosProfile::QOS_TYPE_BOTH)
            {
                msg::TopicQosProfile tQoSMsg;
                tQoSMsg.topic_name = request->topic_name;
                tQoSMsg.qos_type = msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION;
                tQoSMsg.qos_profile = CvtRmwQoSToMsg(qos[msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION]);
                response->topic_qos_profile_vec.push_back(tQoSMsg);
            }
        }
    }
    catch (const TopicQoSException& e)
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosReqSrvCb] Request: %s [%s] failed: %s.", 
            request->topic_name.c_str(), 
            GetQosTypeStr(request->qos_type).c_str(), 
            TopicQoSExceptionMsg[e]);
        response->response = ServiceResponseStatus::SRV_RES_ERROR;
        response->reason = TopicQoSExceptionMsg[e];
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosReqSrvCb] Response: %s [%s] (found: %s).", 
        request->topic_name.c_str(), 
        GetQosTypeStr(request->qos_type).c_str(), 
        GetServiceResponseStatusMsg(response->response));
}



/**
 * Topic device management.
 */



void QoSServer::_topicDevInfoRegSrvCb(const std::shared_ptr<srv::TopicDeviceInfoReg::Request> request, 
                                    std::shared_ptr<srv::TopicDeviceInfoReg::Response> response)
{
    std::lock_guard<std::mutex> topicDevInfoRegSrvLock(mTopicDevInfoRegSrvMtx_);
    std::lock_guard<std::mutex> topicDevInfoStatusMapLock(mTopicDevInfoStatusMapMtx_);
    mTopicDevInfoStatusMap_[request->request.node_name] = { request->request, true };
    RCLCPP_INFO(this->get_logger(), "[QoSServer::_topicDevInfoRegSrvCb] %-30s: %-20s[%-12s] registered.", 
        request->request.node_name.c_str(), 
        request->request.topic_name.c_str(), 
        GetQosTypeStr(request->request.qos_type).c_str());

    // Store to JSON file.
    this->_dumpISrvToJSON(false);
    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;
}



void QoSServer::_qosSrvCb(const std::shared_ptr<srv::QosServer::Request> request, std::shared_ptr<srv::QosServer::Response> response)
{
    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;

    // Qmap action.
    if (request->request.qmap_action == msg::QosServerStatus::QMAP_ACTION_SAVE_FILE)
    {
        if (!this->dumpQmapToJSON())
        {
            response->response = ServiceResponseStatus::SRV_RES_ERROR;
            response->reason = "Failed to save QoS profile map to JSON file.";
        }
    }
    else if (request->request.qmap_action == msg::QosServerStatus::QMAP_ACTION_REMOVE_FILE)
    {
        if (!fs::exists(mQoSFilePath_))
        {
            response->response = ServiceResponseStatus::SRV_RES_ERROR;
            response->reason = "QoS profile map JSON file not found.";
        }
        else
        {
            if (!fs::remove(mQoSFilePath_))
            {
                response->response = ServiceResponseStatus::SRV_RES_ERROR;
                response->reason = "Failed to remove QoS profile map JSON file.";
            }
        }
    }

    // Topic device action.
    if (request->request.topic_device_action == msg::QosServerStatus::TOPIC_DEVICE_ACTION_SAVE_FILE)
    {
        if (!this->_dumpISrvToJSON(true))
        {
            response->response = ServiceResponseStatus::SRV_RES_ERROR;
            response->reason = "Failed to save topic device information map to JSON file.";
        }
    }
    else if (request->request.topic_device_action == msg::QosServerStatus::TOPIC_DEVICE_ACTION_REMOVE_FILE)
    {
        if (fs::exists(mISrvFilePath_))
        {
            if (!fs::remove(mISrvFilePath_))
            {
                response->response = ServiceResponseStatus::SRV_RES_ERROR;
                response->reason = "Failed to remove topic device information map JSON file.";
            }
        }
        else
        {
            response->response = ServiceResponseStatus::SRV_RES_WARNING;
            response->reason = "Topic device information map JSON file not found.";
        }
    }

    // Re-register action.
    if (request->request.re_register_action != msg::QosServerStatus::RE_REGISTER_ACTION_NONE)
    {
        auto topicDevInfoStatusMapCopy = SafeLoad(&mTopicDevInfoStatusMap_, mTopicDevInfoStatusMapMtx_);
        std::set<std::string> reRegNameVec;
        for (const auto& [nodeName, tPair] : topicDevInfoStatusMapCopy)
        {
            const auto& [tInfo, status] = tPair;
            if (request->request.re_register_action == msg::QosServerStatus::RE_REGISTER_ACTION_UNREGISTERED && status)
                continue;
            if (tInfo.manage_qos_node == "")
                continue;
            reRegNameVec.insert(tInfo.manage_qos_node);
        }

        if (reRegNameVec.empty())
        {
            response->response = ServiceResponseStatus::SRV_RES_WARNING;
            response->reason = "No device will be re-registered.";
            return;
        }

        auto req = std::make_shared<srv::InteractiveService::Request>();
        req->device_id = mServiceName_;
        req->service_command_name = "qos_rereg";
        req->service_command_args = CvtReRegisterSignalToInteractiveServiceCommandArgs(0);

        std::map<std::string, std::future<bool> > futureMap;
        for (const auto& nodeName : reRegNameVec)
        {
            futureMap[nodeName] = std::async(std::launch::async, &QoSServer::_sendInteractiveServiceRequest, this, nodeName, req);
        }

        for (auto& [nodeName, future] : futureMap)
        {
            if (future.get())
                RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosSrvCb] Re-register %s done.", nodeName.c_str());
            else
            {
                response->response = ServiceResponseStatus::SRV_RES_ERROR;
                response->reason = "Failed to re-register " + nodeName + ".";
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosSrvCb] Re-register %s failed.", nodeName.c_str());
            }
        }
    }
}



/**
 * InteractiveService management.
 */



bool QoSServer::_sendInteractiveServiceRequest(const std::string& serviceName, const std::shared_ptr<srv::InteractiveService::Request> request)
{
    auto res = ClientRequestHelperRawPtr<srv::InteractiveService>(this, serviceName, request, 500ms);
    if (res)
    {
        if (res->response)
            return true;
        else
            RCLCPP_WARN(this->get_logger(), "[QoSNode::_sendInteractiveServiceRequest] Failed to send request to %s: %s.", serviceName.c_str(), res->reason.c_str());
    }
    return false;
}



bool QoSServer::_sendAllInteracticeNodeTargetAliveStatus(const std::deque<msg::TopicDeviceInfo> &devQue, uint8_t status)
{
    for (const auto& tInfo : devQue)
    {
        auto request = std::make_shared<srv::InteractiveService::Request>();
        request->device_id = mServiceName_;
        request->interactive_service.target_alive = status;
        if (this->_sendInteractiveServiceRequest(tInfo.node_name, request))
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_sendAllInteracticeNodeTargetAliveStatus] Send target alive signal [%d] to %s done.", status, tInfo.node_name.c_str());
        else
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_sendAllInteracticeNodeTargetAliveStatus] Send target alive signal [%d] to %s failed.", status, tInfo.node_name.c_str());
            return false;
        }
    }
    return true;
}



bool QoSServer::_updateInteractiveServiceQoS()
{
    // Get topic device map.
    auto topicDevInfoStatusMapCopy = SafeLoad(&mTopicDevInfoStatusMap_, mTopicDevInfoStatusMapMtx_);
    // Get Current QoS map.
    auto topicQoSMapCopy = SafeLoad(&mTopicQoSMap_, mTopicQoSMapMtx_);
    // Get update map.
    auto topicQoSUpdateMapCopy = SafeLoad(&mTopicQoSUpdateMap_, mTopicQoSUpdateMapMtx_);

    // Group topic devices by QoS profile.
    std::map<TopicQoS, std::deque<msg::TopicDeviceInfo> > qosDevMap;
    for (const auto& [nodeName, tPair] : topicDevInfoStatusMapCopy)// Topic device used to be registered.
    {
        const auto& [tInfo, status] = tPair;
        if (topicQoSUpdateMapCopy.find(tInfo.topic_name) == topicQoSUpdateMapCopy.end())// Topic no need to update.
            continue;
        if (!status)// Topic device on list but not registered.
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Topic device %s [%s] not registered.", 
                tInfo.node_name.c_str(), 
                GetQosTypeStr(tInfo.qos_type).c_str());
            continue;
        }
        qosDevMap[topicQoSMapCopy[tInfo.topic_name]].push_back(tInfo);// Add the topic device under the QoS profile.
    }

    RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Update QoS profile for the topic devices.");
    HierarchicalPrint hprint;
    for (const auto& [tQoS, tInfoQue] : qosDevMap)
    {
        hprint.push(0, "[Topic: %s]", tQoS.getTopicName().c_str());
        for (const auto& tInfo : tInfoQue)
            hprint.push(1, "%s [%s]", tInfo.node_name.c_str(), GetQosTypeStr(tInfo.qos_type).c_str());
    }
    hprint.print();
    hprint.clear();

    // Start to update QoS profile for the topic devices.
    for (const auto& [qos, devQue] : qosDevMap)// Each QoS profile and its devices.
    {
        // Send target alive disable signal to the InteractiveService to close the publisher and subscriber.
        if (!this->_sendAllInteracticeNodeTargetAliveStatus(devQue, msg::InteractiveService::TARGET_ALIVE_DISABLE))
        {
            // Send target alive disable signal failed, recover all nodes.
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Send target alive disable signal to all nodes of topic %s failed.", qos.getTopicName().c_str());
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Recover all nodes of topic %s.", qos.getTopicName().c_str());
            this->_sendAllInteracticeNodeTargetAliveStatus(devQue, msg::InteractiveService::TARGET_ALIVE_ENABLE);
            continue;// Ignore rest of the process and continue to the next QoS profile.
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Send target alive disable signal to all nodes of topic %s done.", qos.getTopicName().c_str());

        //Check whether all publishers and subscribers are closed.
        std::chrono::duration<double, std::milli> checkTopicNodeTimeout(500);// timeout 500ms.
        auto checkTopicNodeStart = std::chrono::high_resolution_clock::now();
        bool allClosed = false;
        while (std::chrono::high_resolution_clock::now() - checkTopicNodeStart < checkTopicNodeTimeout)
        {
            if (this->get_publishers_info_by_topic(qos.getTopicName()).size() <= 0 && 
                this->get_subscriptions_info_by_topic(qos.getTopicName()).size() <= 0)
            {
                allClosed = true;
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
        // If not all nodes are closed, send all nodes the target alive enable signal to open the publisher and subscriber, then continue to the next QoS profile.
        if (!allClosed)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Check topic %s timeout. Not all nodes are closed.", qos.getTopicName().c_str());
            this->_sendAllInteracticeNodeTargetAliveStatus(devQue, msg::InteractiveService::TARGET_ALIVE_ENABLE);
            continue;// Ignore rest of the process and continue to the next QoS profile.
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Check topic %s done. All nodes are closed.", qos.getTopicName().c_str());

        // Send service command to the InteractiveService to update QoS profile for the publisher and subscriber.
        for (const auto& tInfo : devQue)
        {
            auto request = std::make_shared<srv::InteractiveService::Request>();
            request->device_id = mServiceName_;
            request->service_command_name = "qos_update";
            request->service_command_args = CvtRMWQoSToInteractiveServiceCommandArgs(qos[tInfo.qos_type]);
            if (!this->_sendInteractiveServiceRequest(tInfo.node_name, request))
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Send %s command to %s failed.", request->service_command_name.c_str(), tInfo.node_name.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Send %s command to %s done.", request->service_command_name.c_str(), tInfo.node_name.c_str());
        }

        // Send target alive enable signal to the InteractiveService to open the publisher and subscriber.
        if (!this->_sendAllInteracticeNodeTargetAliveStatus(devQue, msg::InteractiveService::TARGET_ALIVE_ENABLE))
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Send target alive enable signal to all nodes of topic %s failed.", qos.getTopicName().c_str());
            continue;// Ignore rest of the process and continue to the next QoS profile.
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Send target alive enable signal to all nodes of topic %s done.", qos.getTopicName().c_str());

        // If topic qos update finished, remove topic name from update map.
        {
            std::lock_guard<std::mutex> topicQoSUpdateMapLock(mTopicQoSUpdateMapMtx_);
            mTopicQoSUpdateMap_.erase(qos.getTopicName());
        }
    }

    // If no error occurs, the topic QoS update map should be empty.
    {
        std::lock_guard<std::mutex> topicQoSUpdateMapLock(mTopicQoSUpdateMapMtx_);
        if (mTopicQoSUpdateMap_.size() > 0)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] Topic QoS update map is not empty.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveServiceQoS] All topic QoS update done.");
        return true;
    }
}


/**
 * Temporary QoS profile management.
 */



bool QoSServer::_topicQoSUpdateCheck(std::map<std::string, std::pair<bool, bool> >& updateMap)// { topicName : { pubF, subF } }
{
    updateMap.clear();
    auto topicQoSMapCopy = SafeLoad(&mTopicQoSMap_, mTopicQoSMapMtx_);
    auto topicQoSMapTmpCopy = SafeLoad(&mTopicQoSMapTmp_, mTopicQoSMapTmpMtx_);

    bool needUpdateF = false;
    needUpdateF |= topicQoSMapCopy.size() != topicQoSMapTmpCopy.size();// True if different sizes
    for (auto& [topicName, topicProp] : topicQoSMapTmpCopy)
    {
        if (!topicProp)// QoSTmp has invalid QoS profile.
            throw topicName + " has invalid QoS profile. " + topicProp.getErrInfo();

        if (topicQoSMapCopy.find(topicName) == topicQoSMapCopy.end())// QoSTmp has new QoS profile.
        {
            updateMap[topicName] = { true, true };
            continue;
        }
        auto cmp = topicProp % topicQoSMapCopy[topicName];
        switch (cmp)
        {
            case TopicQoSCompareResult::QOS_CMP_SAME:// Same QoS profile won't added to updateMap.
                // updateMap[topicName] = { false, false };
                break;
            case TopicQoSCompareResult::QOS_CMP_PUB_DIFF:
                updateMap[topicName] = { true, false };
                break;
            case TopicQoSCompareResult::QOS_CMP_SUB_DIFF:
                updateMap[topicName] = { false, true };
                break;
            case TopicQoSCompareResult::QOS_CMP_BOTH_DIFF:
                updateMap[topicName] = { true, true };
                break;
            case TopicQoSCompareResult::QOS_CMP_TOPIC_DIFF:// It is weird if this happens.
                updateMap[topicName] = { true, true };
                break;
        }
    }
    needUpdateF |= updateMap.size() > 0;
    return needUpdateF;
}



void QoSServer::_loadISrvFromJSON(bool forceWrite)
{
    nlohmann::json json;
    if (LoadFileFromJSON(mISrvFilePath_, json))
    {
        for (const auto& [nodeName, info] : json.items())
        {
            msg::TopicDeviceInfo tInfo;
            tInfo.node_name = nodeName;
            tInfo.topic_name = info["topic_name"];
            tInfo.qos_type = info["qos_type"];
            tInfo.manage_qos_node = info["manage_qos_node"];

            std::lock_guard<std::mutex> topicDevInfoStatusMapLock(mTopicDevInfoStatusMapMtx_);
            if (forceWrite)
                mTopicDevInfoStatusMap_[nodeName] = { tInfo, false };
            else
            {
                if (mTopicDevInfoStatusMap_.find(nodeName) == mTopicDevInfoStatusMap_.end())
                    mTopicDevInfoStatusMap_[nodeName] = { tInfo, false };
            }
        }
    }
}



bool QoSServer::_dumpISrvToJSON(bool useLock)
{
    if (useLock)
        std::lock_guard<std::mutex> topicDevInfoStatusMapLock(mTopicDevInfoStatusMapMtx_);
    nlohmann::json json;
    for (const auto& [nodeName, tPair] : mTopicDevInfoStatusMap_)
    {
        const auto& [tInfo, status] = tPair;
        json[tInfo.node_name]["topic_name"] = tInfo.topic_name;
        json[tInfo.node_name]["qos_type"] = tInfo.qos_type;
        json[tInfo.node_name]["manage_qos_node"] = tInfo.manage_qos_node;
    }
    return DumpJSONToFile(mISrvFilePath_, json);
}



bool QoSServer::setTmpQoSProfile(const msg::TopicQosProfile& tQoSProf, bool checkFlag)
{
    const auto& topicName = tQoSProf.topic_name;
    const auto& qosType = tQoSProf.qos_type;
    const auto prof = CvtMsgToRmwQoS(tQoSProf.qos_profile);

    // Check invalid topic name or qos type.
    if (checkFlag && (topicName == "" || (qosType != msg::TopicQosProfile::QOS_TYPE_PUBLISHER && 
                                            qosType != msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION && 
                                            qosType != msg::TopicQosProfile::QOS_TYPE_BOTH)))
        return false;
    std::lock_guard<std::mutex> topicQoSMapTmpLock(mTopicQoSMapTmpMtx_);
    if (mTopicQoSMapTmp_.find(topicName) == mTopicQoSMapTmp_.end())
        mTopicQoSMapTmp_[topicName] = TopicQoS(topicName);

    switch (qosType)
    {
        case msg::TopicQosProfile::QOS_TYPE_PUBLISHER:
            mTopicQoSMapTmp_[topicName].setPubQoS(prof);
            break;
        case msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION:
            mTopicQoSMapTmp_[topicName].setSubQoS(prof);
            break;
        case msg::TopicQosProfile::QOS_TYPE_BOTH:
            mTopicQoSMapTmp_[topicName].setQoS(prof);
            break;
    }
    return true;
}



void QoSServer::removeTmpQoSProfile(const std::string& topicName)
{
    std::lock_guard<std::mutex> topicQoSMapTmpLock(mTopicQoSMapTmpMtx_);
    mTopicQoSMapTmp_.erase(topicName);
}



void QoSServer::clearTmpQoSProfile()
{
    std::lock_guard<std::mutex> topicQoSMapTmpLock(mTopicQoSMapTmpMtx_);
    mTopicQoSMapTmp_.clear();
}



void QoSServer::recoverTmpQoSProfile()
{
    std::lock_guard<std::mutex> topicQoSMapLock(mTopicQoSMapMtx_);
    std::lock_guard<std::mutex> topicQoSMapTmpLock(mTopicQoSMapTmpMtx_);
    mTopicQoSMapTmp_ = mTopicQoSMap_;
}



ReasonResult<bool> QoSServer::setTopicQoSMap()
{
    std::map<std::string, std::pair<bool, bool> > updateMap;
    bool needUpdate = false;
    try
    {
        needUpdate = this->_topicQoSUpdateCheck(updateMap);
    }
    catch (const TopicQoSException& e)
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer::setTopicQoSMap] Temporary QoS check failed: %s.", TopicQoSExceptionMsg[e]);
        return { false, TopicQoSExceptionMsg[e] };
    }
    catch (const std::string& e)
    {
        RCLCPP_WARN(this->get_logger(), "[QoSServer::setTopicQoSMap] Temporary QoS check failed: %s.", e.c_str());
        return { false, e };
    }

    if (needUpdate)
    {
        RCLCPP_INFO(this->get_logger(), "[QoSServer::setTopicQoSMap] QoS need updated!");
        for (const auto& [topicName, update] : updateMap)
            RCLCPP_INFO(this->get_logger(), "[QoSServer::setTopicQoSMap] %-30s: %-9s %-12s.", topicName.c_str(), update.first ? "publisher" : "", update.second ? "subscription" : "");

        {// Set the temporary QoS profile map to the QoS profile map.
            std::lock_guard<std::mutex> topicQoSMapLock(mTopicQoSMapMtx_);
            std::lock_guard<std::mutex> topicQoSMapTmpLock(mTopicQoSMapTmpMtx_);

            mTopicQoSMap_ = mTopicQoSMapTmp_;
            mQID_ += 1;
            SafeStore(&mTopicQoSUpdateMap_, updateMap, mTopicQoSUpdateMapMtx_);
        }
        this->dumpQmapToJSON();
    }
    this->_updateInteractiveServiceQoS();
    return true;
}



bool QoSServer::loadQmapFromJSON(bool forceWrite)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(mQoSFilePath_)));
        std::map<std::string, TopicQoS> tmpMap;

        for (const auto& [topicName, topicQoS] : json.items())
        {
            for (const auto& [type, qos] : topicQoS.items())
            {
                rmw_qos_profile_t tmpProf;
                tmpProf.history = (rmw_qos_history_policy_e)qos["history"];
                tmpProf.depth = qos["depth"];
                tmpProf.reliability = (rmw_qos_reliability_policy_e)qos["reliability"];
                tmpProf.durability = (rmw_qos_durability_policy_e)qos["durability"];
                tmpProf.deadline = CvtMsToRmwTime(qos["deadline_ms"]);
                tmpProf.lifespan = CvtMsToRmwTime(qos["lifespan_ms"]);
                tmpProf.liveliness = (rmw_qos_liveliness_policy_e)qos["liveliness"];
                tmpProf.liveliness_lease_duration = CvtMsToRmwTime(qos["liveliness_lease_duration_ms"]);
                tmpMap[topicName].setTopicName(topicName);
                if (type == "publisher")
                    tmpMap[topicName].setPubQoS(tmpProf);
                else if (type == "subscription")
                    tmpMap[topicName].setSubQoS(tmpProf);
            }
        }
        std::lock_guard<std::mutex> topicQoSMapTmpLock(mTopicQoSMapTmpMtx_);
        mTopicQoSMapTmp_ = tmpMap;
        if (forceWrite)
        {
            std::lock_guard<std::mutex> topicQoSMapLock(mTopicQoSMapMtx_);
            mTopicQoSMap_ = tmpMap;
        }
    }
    catch (...)
    {
        return false;
    }
    return true;
}



bool QoSServer::dumpQmapToJSON()
{
    auto topicQoSMapCopy = SafeLoad(&mTopicQoSMap_, mTopicQoSMapMtx_);
    try
    {
        nlohmann::json json;
        for (auto& [topicName, topicProp] : topicQoSMapCopy)
        {
            if (topicProp.getTopicName() == "")
                continue;
            if (topicProp.isPubQoSValid())
            {
                auto& prof = topicProp[msg::TopicQosProfile::QOS_TYPE_PUBLISHER];
                json[topicProp.getTopicName()]["publisher"]["history"] = (int8_t)prof.history;
                json[topicProp.getTopicName()]["publisher"]["depth"] = prof.depth;
                json[topicProp.getTopicName()]["publisher"]["reliability"] = (int8_t)prof.reliability;
                json[topicProp.getTopicName()]["publisher"]["durability"] = (int8_t)prof.durability;
                json[topicProp.getTopicName()]["publisher"]["deadline_ms"] = CvtRmwTimeToMs(prof.deadline);
                json[topicProp.getTopicName()]["publisher"]["lifespan_ms"] = CvtRmwTimeToMs(prof.lifespan);
                json[topicProp.getTopicName()]["publisher"]["liveliness"] = (int8_t)prof.liveliness;
                json[topicProp.getTopicName()]["publisher"]["liveliness_lease_duration_ms"] = CvtRmwTimeToMs(prof.liveliness_lease_duration);
            }
            if (topicProp.isSubQoSValid())
            {
                auto& prof = topicProp[msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION];
                json[topicProp.getTopicName()]["subscription"]["history"] = (int8_t)prof.history;
                json[topicProp.getTopicName()]["subscription"]["depth"] = prof.depth;
                json[topicProp.getTopicName()]["subscription"]["reliability"] = (int8_t)prof.reliability;
                json[topicProp.getTopicName()]["subscription"]["durability"] = (int8_t)prof.durability;
                json[topicProp.getTopicName()]["subscription"]["deadline_ms"] = CvtRmwTimeToMs(prof.deadline);
                json[topicProp.getTopicName()]["subscription"]["lifespan_ms"] = CvtRmwTimeToMs(prof.lifespan);
                json[topicProp.getTopicName()]["subscription"]["liveliness"] = (int8_t)prof.liveliness;
                json[topicProp.getTopicName()]["subscription"]["liveliness_lease_duration_ms"] = CvtRmwTimeToMs(prof.liveliness_lease_duration);
            }
        }
        std::ofstream outFile(mQoSFilePath_);
        outFile << json;
    }
    catch (...)
    {
        return false;
    }
    return true;
}

}// namespace rv2_interfaces



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rv2_interfaces::QoSServer)

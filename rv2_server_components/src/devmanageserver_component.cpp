#include <rv2_server_components/devmanageserver_component.h>

namespace rv2_interfaces
{

/**
 * ================================================================
 * DevManageServer Constructor and Destructor
 * ================================================================
 */



DevManageServer::DevManageServer(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(DEFAULT_DEVMANAGE_SERVER_NODENAME, options), 
    mStoreStrategy_(NodeAddrStoreStrategy::CONFLICT_OVERWRITE)
{
    // Get parameters
    int nodeAddrStoreStrategy = 0;
    double procExpTimeout_ms = 1000.0;
    GetParamRawPtr(this, "devManageService", mDevManageSrvName_, mDevManageSrvName_, "devManageService: ", false);
    GetParamRawPtr(this, "devManageServerDirPath", mDevInfoDirPathName_, mDevInfoDirPathName_, "devManageServerDirPath: ", false);
    GetParamRawPtr(this, "nodeAddrStoreStrategy", nodeAddrStoreStrategy, nodeAddrStoreStrategy, "nodeAddrStoreStrategy: ", false);
    GetParamRawPtr(this, "procExpTimeout_ms", procExpTimeout_ms, procExpTimeout_ms, "procExpTimeout_ms: ", false);
    mDevInfoDirPath_ = mDevInfoDirPathName_;
    mStoreStrategy_ = (NodeAddrStoreStrategy)nodeAddrStoreStrategy;
    mProcExpTimeout_ns_ = std::chrono::milliseconds((int64_t)procExpTimeout_ms);

    fs::create_directories(mDevInfoDirPath_);

    this->_loadNodeAddrFile();
    this->printNodeAddrList();

    mPSRegSrv_ = this->create_service<srv::ProcStatusReg>(DevManageServer_ProcStatusRegSrvName(mDevManageSrvName_), 
        std::bind(&DevManageServer::_PSRegSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    mNodeAddrRegSrv_ = this->create_service<srv::NodeAddrReg>(DevManageServer_NodeAddrRegSrvName(mDevManageSrvName_), 
        std::bind(&DevManageServer::_nodeAddrRegSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    mNodeAddrReqSrv_ = this->create_service<srv::NodeAddrReq>(DevManageServer_NodeAddrReqSrvName(mDevManageSrvName_), 
        std::bind(&DevManageServer::_nodeAddrReqSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    mDevInfoReqSrv_ = this->create_service<srv::DevInfoReq>(DevManageServer_DevInfoReqSrvName(mDevManageSrvName_), 
        std::bind(&DevManageServer::_devInfoReqSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    mDevManageSrv_ = this->create_service<srv::DevManageServer>(DevManageServer_DevManageServerSrvName(mDevManageSrvName_), 
        std::bind(&DevManageServer::_devManageSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "[DevManageServer] Constructed.");
}



DevManageServer::~DevManageServer()
{
    RCLCPP_INFO(this->get_logger(), "[DevManageServer] Destructed.");
}



/**
 * ================================================================
 * DevManageServer Private Methods
 * ================================================================
 */



void DevManageServer::_PSRegSrvCb(const std::shared_ptr<srv::ProcStatusReg::Request> request, 
                                    std::shared_ptr<srv::ProcStatusReg::Response> response)
{
    std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
    if (mDevManageMap_.find(request->node_name) != mDevManageMap_.end())
    {
        mDevManageMap_[request->node_name].ps = request->proc_status_vec;
        mDevManageMap_[request->node_name].lastReqTime = std::chrono::system_clock::now();
        response->response = ServiceResponseStatus::SRV_RES_SUCCESS;
        return;
    }
    response->response = ServiceResponseStatus::SRV_RES_IGNORED;
}



void DevManageServer::_nodeAddrRegSrvCb(const std::shared_ptr<srv::NodeAddrReg::Request> request, 
                                        std::shared_ptr<srv::NodeAddrReg::Response> response)
{
    response->response = ServiceResponseStatus::SRV_RES_IGNORED;
    if (request->node_addr.node_name.length() <= 0)
    {
        response->reason = "[_nodeAddrRegSrvCb] Node name is empty.";
        return;
    }
    std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
    try
    {
        RCLCPP_INFO(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Register: %s", GetNodeAddrStr(request->node_addr).c_str());

        std::vector<std::string> conflictKeyVec;
        for (const auto& [n, dm] : mDevManageMap_)
        {
            if (CheckNodeAddrConflict(dm.nodeAddr, request->node_addr) < 0)// Conflict or Conflict node name.
            {
                RCLCPP_WARN(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Conflict: %s", dm.nodeAddr.node_name.c_str());
                conflictKeyVec.push_back(n);
            }
        }

        if (conflictKeyVec.size() > 0)
        {
            if (mStoreStrategy_ == NodeAddrStoreStrategy::CONFLICT_OVERWRITE)
            {
                for (const auto& cKey : conflictKeyVec)// Remove conflict files and elements
                {
                    // Remove conflict file
                    auto jsonFilePath = this->_nodeNameToJSONFilePath(cKey);// Current file conflicts with new NodeAddr.
                    if (fs::exists(jsonFilePath))
                        fs::remove(jsonFilePath);// Remove current file.
                    auto jsonConflictFilePath = this->_nodeNameToJSONFilePath(cKey, true);// Current NodeAddr dump to conflicted file.
                    nlohmann::json json;
                    if (CvtNodeAddrToJSON(mDevManageMap_[cKey].nodeAddr, json) && DumpJSONToFile(jsonConflictFilePath, json))
                    {
                        RCLCPP_WARN(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Move conflicted file to %s.", 
                                    jsonConflictFilePath.generic_string().c_str());
                    }

                    // Remove conflict NodeAddr from map.
                    mDevManageMap_.erase(cKey);
                    RCLCPP_WARN(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Remove: %s", cKey.c_str());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Ignored: %s", request->node_addr.node_name.c_str());
                response->response = ServiceResponseStatus::SRV_RES_IGNORED;
                return;
            }
        }
        // Add new NodeAddr to map.
        mDevManageMap_[request->node_addr.node_name].nodeAddr = request->node_addr;
        // Dump file.
        auto jsonFilePath = this->_nodeNameToJSONFilePath(request->node_addr.node_name);
        nlohmann::json json;
        if (CvtNodeAddrToJSON(request->node_addr, json) && DumpJSONToFile(jsonFilePath, json))
        {
            RCLCPP_INFO(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Dumped %s to %s", 
                        request->node_addr.node_name.c_str(), 
                        jsonFilePath.generic_string().c_str());
        }
        response->response = ServiceResponseStatus::SRV_RES_SUCCESS;
    }
    catch(const std::exception& e)
    {
        response->response = ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;
        RCLCPP_ERROR(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] %s", e.what());
    }
    catch (...)
    {
        response->response = ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;
        RCLCPP_ERROR(this->get_logger(), "[DevManageServer::_nodeAddrRegSrvCb] Unknown exceptions");
    }
}



void DevManageServer::_nodeAddrReqSrvCb(const std::shared_ptr<srv::NodeAddrReq::Request> request, 
                                        std::shared_ptr<srv::NodeAddrReq::Response> response)
{
    auto tmp = SafeLoad(&mDevManageMap_, mDevManageMapMtx_);

    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;
    try
    {
        if (request->node_addr.node_name == "all")
        {
            for (const auto& [n, dm] : tmp)
                response->node_addr_vec.push_back(dm.nodeAddr);
        }
        else
        {
            NodeAddrIndicators indicators;
            for (const auto& [n, dm] : tmp)
            {
                if (IsNodeAddrIntersect(dm.nodeAddr, request->node_addr, indicators))
                    response->node_addr_vec.push_back(dm.nodeAddr);
            }
        }
    }
    catch(const std::exception& e)
    {
        response->response = ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;
        RCLCPP_ERROR(this->get_logger(), "[DevManageServer::_nodeAddrReqSrvCb] %s", e.what());
    }
    catch (...)
    {
        response->response = ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;
        RCLCPP_ERROR(this->get_logger(), "[DevManageServer::_nodeAddrReqSrvCb] Unknown exceptions");
    }
}



void DevManageServer::_devInfoReqSrvCb(const std::shared_ptr<srv::DevInfoReq::Request> request, 
                        std::shared_ptr<srv::DevInfoReq::Response> response)
{
    auto tmp = SafeLoad(&mDevManageMap_, mDevManageMapMtx_);

    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;
    if (request->node_addr.node_name == "all")
    {
        for (const auto& [n, dm] : tmp)
        {
            response->dev_info_vec.push_back(std::move(this->_cvtDevManageStructToDevInfo(dm)));
        }
    }
    else
    {
        NodeAddrIndicators indicators;
        for (const auto& [n, dm] : tmp)
        {
            if (IsNodeAddrIntersect(dm.nodeAddr, request->node_addr, indicators))
                response->dev_info_vec.push_back(std::move(this->_cvtDevManageStructToDevInfo(dm)));
        }
    }
}



void DevManageServer::_devManageSrvCb(const std::shared_ptr<srv::DevManageServer::Request> request, 
                        std::shared_ptr<srv::DevManageServer::Response> response)
{
    response->response = ServiceResponseStatus::SRV_RES_SUCCESS;
    // Node address action
    if (request->request.node_addr_action == msg::DevManageServerStatus::NODE_ADDR_ACTION_ADD)
    {
        auto tmpReq = std::make_shared<srv::NodeAddrReg::Request>();
        auto tmpRes = std::make_shared<srv::NodeAddrReg::Response>();
        tmpReq->node_addr = request->request.node_addr;
        this->_nodeAddrRegSrvCb(tmpReq, tmpRes);
        response->response = tmpRes->response;
        response->reason += tmpRes->reason;
    }
    else if (request->request.node_addr_action == msg::DevManageServerStatus::NODE_ADDR_ACTION_REMOVE)
    {
        auto tmp = SafeLoad(&mDevManageMap_, mDevManageMapMtx_);

        std::vector<std::string> removeKeyVec;
        for (const auto& [n, dm] : tmp)
        {
            NodeAddrIndicators indicators;
            if (IsNodeAddrIntersect(dm.nodeAddr, request->request.node_addr, indicators))
                removeKeyVec.push_back(n);
        }

        for (const auto& k : removeKeyVec)
        {
            auto jsonFilePath = this->_nodeNameToJSONFilePath(k);
            if (fs::exists(jsonFilePath))
                fs::remove(jsonFilePath);

            std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
            mDevManageMap_.erase(k);
        }
    }

    // Device management service action
    if (request->request.server_action & msg::DevManageServerStatus::SERVER_ACTION_SET_TIMER)
    {
        response->response = ServiceResponseStatus::SRV_RES_IGNORED;
    }

    if (request->request.server_action & msg::DevManageServerStatus::SERVER_ACTION_SET_PERIOD)
    {
        response->response = ServiceResponseStatus::SRV_RES_IGNORED;
    }

    if (request->request.server_action & msg::DevManageServerStatus::SERVER_ACTION_KILL_NODE)
    {
        const auto& req = request->request;

        if (req.server_kill_node_name.length() <= 0)
        {
            response->reason += "[KILL_NODE] Node name is empty.";
            return;
        }
        DevManageForceExit prof;
        prof.exit_code = 1;
        prof.delay_ms = req.server_kill_node_delay_ns > 0 ?  RCUTILS_NS_TO_MS(req.server_kill_node_delay_ns) : 0;
        auto request = std::make_shared<srv::InteractiveService::Request>();
        request->device_id = mDevManageSrvName_;
        request->service_command_name = "node_exit";
        request->service_command_args = CvtDevManageForceExitToInteractiveServiceCommandArgs(prof);
        auto res = ClientRequestHelperRawPtr<srv::InteractiveService>(this, DevManageNode_ISrvName(req.server_kill_node_name), request, 500ms);
        if (res)
        {
            if (!res->response) response->reason += res->reason;
        }
        else
        {
            response->response = ServiceResponseStatus::SRV_RES_WARNING;
            response->reason += "[KILL_NODE] InteractiveService request failed.";
        }
    }
}



std::map<std::string, DevManageServer::devManageStruct_> DevManageServer::_getDevManageMap(std::string nodeName)
{
    std::map<std::string, DevManageServer::devManageStruct_> ret;
    std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
    if (nodeName == "")
        return mDevManageMap_;
    if (mDevManageMap_.find(nodeName) != mDevManageMap_.end())
        ret[nodeName] = mDevManageMap_[nodeName];
    return ret;
}



msg::DevInfo DevManageServer::_cvtDevManageStructToDevInfo(const DevManageServer::devManageStruct_& dm)
{
    msg::DevInfo ret;
    ret.node_addr = dm.nodeAddr;
    for (const auto& ps : dm.ps)
        ret.proc_status_vec.push_back(ps);
    ret.is_proc_status_expired = std::chrono::system_clock::now() - dm.lastReqTime > mProcExpTimeout_ns_;
    return ret;
}



void DevManageServer::_loadNodeAddrFile()
{
    std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
    for (auto& fp : fs::directory_iterator(mDevInfoDirPath_))
    {
        if (fp.path().extension() == ".conflict")
            continue;
        auto msg = msg::NodeAddr();
        nlohmann::json json;
        printf("Found: %s\n", fp.path().generic_string().c_str());
        if (LoadFileFromJSON(fp, json) && CvtJSONToNodeAddr(json, msg))
            mDevManageMap_[msg.node_name].nodeAddr = msg;
    }
}



fs::path DevManageServer::_nodeNameToJSONFilePath(const std::string& nodeName, bool conflict)
{
    std::string fn = nodeName;
    rv2_interfaces::replace_all(fn, "/", "_");
    fn += (conflict ? ".json.conflict" : ".json");
    return mDevInfoDirPath_ / fn;
}



/**
 * ================================================================
 * DevManageServer Public Methods
 * ================================================================
 */



std::map<std::string, msg::NodeAddr> DevManageServer::getNodeAddrList()
{
    std::map<std::string, msg::NodeAddr> ret;
    std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
    for (const auto& [n, dm] : mDevManageMap_)
        ret[n] = dm.nodeAddr;
    return ret;
}



bool DevManageServer::sendExitRequest(const std::string& nodeName, int delay_ns)
{
    DevManageForceExit prof;
    prof.exit_code = 1;
    prof.delay_ms = RCUTILS_NS_TO_MS(delay_ns);
    auto request = std::make_shared<srv::InteractiveService::Request>();
    request->device_id = mDevManageSrvName_;
    request->service_command_name = "node_exit";
    request->service_command_args = CvtDevManageForceExitToInteractiveServiceCommandArgs(prof);
    auto res = ClientRequestHelperRawPtr<srv::InteractiveService>(this, nodeName, request, 500ms);
    if (res)
        return res->response;
    return false;
}



void DevManageServer::printNodeAddrList()
{
    std::lock_guard<std::mutex> devManageMapLock(mDevManageMapMtx_);
    printf("====Device Information List====\n");
    for (const auto& [n, dm] : mDevManageMap_)
    {
        printf("%s\n", GetNodeAddrStr(dm.nodeAddr).c_str());
    }
    printf("===============================\n");
}



void DevManageServer::printProcedureStatus(std::string nodeName)
{
    auto dmMap = this->_getDevManageMap(nodeName);
    auto printTs = std::chrono::system_clock::now();
    HierarchicalPrint hprint;
    for (const auto& [n, dm] : dmMap)
        AddHierarchicalPrint(hprint, 0, this->_cvtDevManageStructToDevInfo(dm));
    printf("====Procedure Status List====\n");
    hprint.print();
    printf("=============================\n");
    hprint.clear();
}

}// namespace rv2_interfaces



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rv2_interfaces::DevManageServer)

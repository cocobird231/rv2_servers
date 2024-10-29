#pragma once
#include <rv2_interfaces/rv2_interfaces.h>
#include <rv2_interfaces/visibility_control.h>

#define DEFAULT_QOS_SRV_NAME "qos_default"
#define DEFAULT_QOS_SERVER_NODENAME "qosserver_default"
#define DEFAULT_QOS_SERVER_DIR_PATH "qosserver"

namespace rv2_interfaces
{

/**
 * ============================================================================
 * QoSServer Class Definition
 * ============================================================================
 */

/**
 *      The class implements the QoS management service.
 *      The QoSServer manages the Registered QoS profiles and the topic device information, 
 *      and provides the QoS profile update mechanism to the QoSNode.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <service_name>                    : The QoSServer control service.
 *          <service_name>_Reg                : The QoS profile registration service.
 *          <service_name>_Req                : The QoS profile request service.
 *          <service_name>_topicdevinfo_Reg   : The topic device information registration service.
 */
class QoSServer : public rclcpp::Node
{
private:
    // Parameters
    std::string mServiceName_ = DEFAULT_QOS_SRV_NAME;// The name of the QoSServer service.
    std::string mQoSServerDirPath_ = DEFAULT_QOS_SERVER_DIR_PATH;// The directory path of the QoS configuration files.
    fs::path mQoSFilePath_;
    fs::path mISrvFilePath_;

    /**
     * QoS profile service management.
     */

    // QoS registration and request services.
    rclcpp::Service<srv::TopicQosProfileReg>::SharedPtr mQoSRegSrv_;// The service to receive the QoS profile registration.
    std::mutex mQoSRegSrvMtx_;// The mutex to lock the QoS registration callback function.
    rclcpp::Service<srv::TopicQosProfileReq>::SharedPtr mQoSReqSrv_;// The service to receive the QoS profile request.
    std::mutex mQoSReqSrvMtx_;// The mutex to lock the QoS request callback function.

    /**
     * Topic device management.
     */

    // Topic device registration service.
    rclcpp::Service<srv::TopicDeviceInfoReg>::SharedPtr mTopicDevInfoRegSrv_;// The service to receive the topic device requisition from the QoSNode.
    std::mutex mTopicDevInfoRegSrvMtx_;// The mutex to lock the callack function.

    /**
     * QoS service.
     */
    rclcpp::Service<srv::QosServer>::SharedPtr mQoSSrv_;// The service to control QoS server behaviors.

    // Topic device information.

    // The map to store the topic device information and register status by the node name. 
    // { node_name, { topic_device_info, registered } }
    std::map<std::string, std::pair<msg::TopicDeviceInfo, bool> > mTopicDevInfoStatusMap_;
    std::mutex mTopicDevInfoStatusMapMtx_;// The mutex to lock the mTopicDevInfoStatusMap_.

    // QoS management.
    std::map<std::string, TopicQoS> mTopicQoSMap_;// The map to store the QoS profiles by the topic name.
    std::mutex mTopicQoSMapMtx_;// The mutex to lock the QoS profile map.
    std::map<std::string, TopicQoS> mTopicQoSMapTmp_;// The map to store the temporary QoS profiles by the topic name.
    std::mutex mTopicQoSMapTmpMtx_;// The mutex to lock the temporary QoS profile map.
    std::atomic<uint64_t> mQID_;// The unique ID for whole QoS profiles.
    std::map<std::string, std::pair<bool, bool> > mTopicQoSUpdateMap_;// The map to store the topic QoS update status: <topicName, <pub, sub>>.
    std::mutex mTopicQoSUpdateMapMtx_;// The mutex to lock the topic QoS update status map.

public:
    COMPOSITION_PUBLIC
    explicit QoSServer(const rclcpp::NodeOptions & options);

private:
    /**
     * QoS profile registration and requisition service.
     */

    /**
     * @brief QoS registration service callback function.
     * @details The function is called when the QoS registration service receives a request.
     * @note The function is using mTopicQoSMapMtx_ and mTopicQoSMapTmpMtx_.
     */
    void _qosRegSrvCb(const std::shared_ptr<srv::TopicQosProfileReg::Request> request, 
                                std::shared_ptr<srv::TopicQosProfileReg::Response> response);

    /**
     * @brief Response the QoS profile request. TODO: to be refined.
     * @details The function responses the QoS profile by the topic name and the type "publisher" or "subscription".
     * @note The function is using mTopicQoSMapMtx_.
     */
    void _qosReqSrvCb(const std::shared_ptr<srv::TopicQosProfileReq::Request> request, 
                                std::shared_ptr<srv::TopicQosProfileReq::Response> response);

    /**
     * Topic device management.
     */

    /**
     * @brief Topic device information registration service callback function.
     * @details The function is called when the QoSNode sends the topic device information registration request.
     * @note The function is using mTopicDevInfoStatusMapMtx_.
     */
    void _topicDevInfoRegSrvCb(const std::shared_ptr<srv::TopicDeviceInfoReg::Request> request, 
                                        std::shared_ptr<srv::TopicDeviceInfoReg::Response> response);

    void _qosSrvCb(const std::shared_ptr<srv::QosServer::Request> request, std::shared_ptr<srv::QosServer::Response> response);



    /**
     * InteractiveService management.
     */

    /**
     * @brief Send request to the InteractiveService.
     * @details The function sends the request to the InteractiveService.
     * @param[in] serviceName The name of the InteractiveService.
     * @param[in] request The request to send.
     * @return True if the request is sent successfully; false otherwise.
     */
    bool _sendInteractiveServiceRequest(const std::string& serviceName, const std::shared_ptr<srv::InteractiveService::Request> request);

    /**
     * @brief Send target alive signal to all InteractiveServices.
     * @details The function sends the target alive signal to all InteractiveServices.
     * @param[in] devQue The deque of the topic device information.
     * @param[in] status The target alive status.
     * @return True if all target alive signal are sent successfully; false otherwise.
     * @note The function will stop if any target alive signal is sent failed.
     */
    bool _sendAllInteracticeNodeTargetAliveStatus(const std::deque<msg::TopicDeviceInfo> &devQue, uint8_t status);

    /**
     * @brief Update QoS profile for the topic devices.
     * @details The function updates the QoS profile for the topic devices.
     * @return True if the QoS profile is updated successfully; false otherwise.
     * @note The functin will called InteractiveService directly to update the QoS profile for the publisher and subscriber.
     * @note The function is using mTopicDevInfoStatusMapMtx_, mTopicQoSMapMtx_ and mTopicQoSUpdateMapMtx_.
     */
    bool _updateInteractiveServiceQoS();


    /**
     * Temporary QoS profile management.
     */

    /**
     * @brief Check temporary QoS profile update status.
     * @details The function checks the temporary QoS profile with the current QoS profile.
     * @param[out] updateMap The map to store the update status: <topicName, <pub, sub>>.
     * @return True if the temporary QoS profile is different from the current QoS profile; false otherwise.
     * @note The function is using mTopicQoSMapMtx_ and mTopicQoSMapTmpMtx_.
     * @note The function will throw TopicQoSException if the temporary QoS profile is invalid.
     */
    bool _topicQoSUpdateCheck(std::map<std::string, std::pair<bool, bool> >& updateMap);// { topicName : { pubF, subF } }

    void _loadISrvFromJSON(bool forceWrite = false);

    bool _dumpISrvToJSON(bool useLock = true);

public:
    /**
     * @brief Set the QoS profile to the temporary QoS profile map.
     * @details The function sets the QoS profile to the temporary QoS profile map by the topic name and the node type.
     * @param[in] topicName The name of the topic.
     * @param[in] qosType The type of the qos profile: "publisher", "subscription", or "both".
     * @param[in] prof The QoS profile.
     * @return True if the QoS profile is set; false otherwise.
     * @note The function is using mTopicQoSMapTmpMtx_.
     */
    bool setTmpQoSProfile(const msg::TopicQosProfile& tQoSProf, bool checkFlag = true);

    /**
     * @brief Remove the temporary QoS profile by the topic name, including both publisher and subscriber QoS profiles.
     * @details The function removes the temporary QoS profile by the topic name, including both publisher and subscriber QoS profiles.
     * @param[in] topicName The name of the topic.
     * @note The function is using mTopicQoSMapTmpMtx_.
     */
    void removeTmpQoSProfile(const std::string& topicName);

    /**
     * @brief Clear the temporary QoS profile map.
     * @details The function clears the temporary QoS profile map.
     * @note The function is using mTopicQoSMapTmpMtx_.
     */
    void clearTmpQoSProfile();

    /**
     * @brief Recover the temporary QoS profile map.
     * @details The function recovers the temporary QoS profile map from the QoS profile map.
     * @note The function is using mTopicQoSMapMtx_ and mTopicQoSMapTmpMtx_.
     */
    void recoverTmpQoSProfile();

    /**
     * @brief Set the temporary QoS profile map to the QoS profile map and dump the QoS profile map to the file.
     * @details The function sets the temporary QoS profile map to the QoS profile map and dumps the QoS profile map to the file.
     * @return True if the QoS profile map is set; false otherwise.
     * @note The function is using mTopicQoSMapMtx_, mTopicQoSMapTmpMtx_, mTopicQoSUpdateMapMtx_ and mTopicDevInfoStatusMapMtx_.
     */
    ReasonResult<bool> setTopicQoSMap();

    /**
     * @brief Load QoS profiles to the temporary QoS profile map from the file.
     * @details The function loads the QoS profiles to the temporary QoS profile map from the file.
     * @param[in] forceWrite Set true to force write both temporary and current QoS profile maps to the file.
     * @return True if the QoS profile is loaded; false otherwise.
     * @note If load failed, the temporary QoS profile map will not be modified.
     * @note The function is using mTopicQoSMapTmpMtx_.
     */
    bool loadQmapFromJSON(bool forceWrite = false);

    /**
     * @brief Dump the QoS profiles map to the file.
     * @details The function dumps the QoS profile map to the file.
     * @return True if the QoS profile map is dumped; false otherwise.
     * @note The function is using mTopicQoSMapMtx_.
     */
    bool dumpQmapToJSON();
};

}

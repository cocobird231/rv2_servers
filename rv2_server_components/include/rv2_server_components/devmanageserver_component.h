#pragma once
#include <rv2_interfaces/rv2_interfaces.h>
#include <rv2_interfaces/visibility_control.h>

#define DEFAULT_DEVMANAGE_SRV_NAME "devmanage_default"
#define DEFAULT_DEVMANAGE_SERVER_NODENAME "devmanageserver_default"
#define DEFAULT_DEVMANAGE_SERVER_DIR_PATH "devmanageserver/nodes"

namespace rv2_interfaces
{

/**
 * ================================================================
 * DevManageServer Class Definition
 * ================================================================
 */

/**
 *      The Device Management Service is used to manage the device information and the procedure status, 
 *      and provide the node exit command to the DevManageNode.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <service_name>                : The DevManageServer control service.
 *          <service_name>_nodeaddr_Reg   : The node address registration service.
 *          <service_name>_nodeaddr_Req   : The node address request service.
 *          <service_name>_devinfo_Req    : The device information request service.
 */
class DevManageServer : public rclcpp::Node
{
private:
    // Parameters
    std::string mDevManageSrvName_ = DEFAULT_DEVMANAGE_SRV_NAME;
    std::string mDevInfoDirPathName_ = DEFAULT_DEVMANAGE_SERVER_DIR_PATH;
    double mProcedureScanPeriod_ms_ = 1000.0;

    rclcpp::Service<srv::NodeAddrReg>::SharedPtr mNodeAddrRegSrv_;// Node address registration service.
    rclcpp::Service<srv::NodeAddrReq>::SharedPtr mNodeAddrReqSrv_;// Node address request service.
    rclcpp::Service<srv::DevInfoReq>::SharedPtr mDevInfoReqSrv_;// Device information request service.
    rclcpp::Service<srv::DevManageServer>::SharedPtr mDevManageSrv_;// Device management service.

    std::shared_ptr<LiteTimer> mPSReqTm_;// Procedure status request timer.
    std::chrono::nanoseconds mPSReqTmPeriod_ns_;// Procedure status request timer period.

    struct devManageStruct_
    {
        msg::NodeAddr nodeAddr;
        std::vector<msg::ProcStatus> ps;
        std::chrono::system_clock::time_point lastReqTime;
    };

    std::map<std::string, devManageStruct_> mDevManageMap_;// { nodeName, devManageStruct_ }
    std::mutex mDevManageMapMtx_;

    fs::path mDevInfoDirPath_;

    enum NodeAddrStoreStrategy { CONFLICT_OVERWRITE, CONFLICT_IGNORE } mStoreStrategy_;

public:
    COMPOSITION_PUBLIC
    explicit DevManageServer(const rclcpp::NodeOptions & options);

private:
    void _nodeAddrRegSrvCb(const std::shared_ptr<srv::NodeAddrReg::Request> request, 
                            std::shared_ptr<srv::NodeAddrReg::Response> response);

    void _nodeAddrReqSrvCb(const std::shared_ptr<srv::NodeAddrReq::Request> request, 
                            std::shared_ptr<srv::NodeAddrReq::Response> response);

    void _devInfoReqSrvCb(const std::shared_ptr<srv::DevInfoReq::Request> request, 
                            std::shared_ptr<srv::DevInfoReq::Response> response);

    void _devManageSrvCb(const std::shared_ptr<srv::DevManageServer::Request> request, 
                            std::shared_ptr<srv::DevManageServer::Response> response);

    void _psReqTmCb();

    std::map<std::string, devManageStruct_> _getDevManageMap(std::string nodeName);

    msg::DevInfo _cvtDevManageStructToDevInfo(const devManageStruct_& dm);

    void _loadNodeAddrFile();

    fs::path _nodeNameToJSONFilePath(const std::string& nodeName, bool conflict = false);

public:
    std::map<std::string, msg::NodeAddr> getNodeAddrList();

    bool sendExitRequest(const std::string& nodeName, int delay_ns);

    void printNodeAddrList();

    void printProcedureStatus(std::string nodeName);
};

}

#include <random>
#include <thread>
#include <iostream>
#include <sstream>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/utils.h>
#include <rv2_interfaces/device_management.h>

#define DEFAULT_SERVICE_NAME "devmanage_default"

static std::atomic<bool> __globalExitFlag = false;

enum DevInfoContentEnum { NODENAME, HOSTNAME, MAC, IPV4, IPV6 };

std::string GenDevInfoContent(DevInfoContentEnum type)
{
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::string ret = "";
    std::stringstream ss;

    std::uniform_int_distribution<> uniInt26Distrib{0, 25};
    std::uniform_int_distribution<> uniInt255Distrib{1, 255};
    std::uniform_int_distribution<> uniInt65535Distrib{1, 255};

    switch (type)
    {
    case DevInfoContentEnum::NODENAME:
        ret += "/V0/";
        for (int i = 0; i < 8; i++)
            ret += uniInt26Distrib(gen_) + 97;
        ret += "_" + std::to_string(uniInt26Distrib(gen_)) + "_node";
        break;
    case DevInfoContentEnum::HOSTNAME:
        ret += "v0-";
        for (int i = 0; i < 8; i++)
            ret += uniInt26Distrib(gen_) + 97;
        ret += "-" + std::to_string(uniInt26Distrib(gen_));
        break;
    case DevInfoContentEnum::MAC:
        for (int i = 0; i < 5; i++)
            ss << std::setfill('0') << std::setw(2) << std::hex << uniInt255Distrib(gen_) << ":";
        ss << std::setfill('0') << std::setw(2) << std::hex << uniInt255Distrib(gen_);
        ret += ss.str();
        break;
    case DevInfoContentEnum::IPV4:
        for (int i = 0; i < 3; i++)
            ret += std::to_string(uniInt255Distrib(gen_)) + ".";
        ret += std::to_string(uniInt255Distrib(gen_));
        break;
    case DevInfoContentEnum::IPV6:
        for (int i = 0; i < 7; i++)
            ss << std::hex << uniInt65535Distrib(gen_) << ":";
        ss << std::hex << uniInt65535Distrib(gen_);
        ret += ss.str();
        break;
    default:
        break;
    }
    return ret;
}

void PrintHelp()
{
    printf("/** \n\
 * Register node address: \n\
 *   g <node_name> <hostname> <ipv4> <mac> \n\
 * \n\
 * Request node address: \n\
 *   qn <node_name> <hostname> <ipv4> <mac> \n\
 *   qn all \n\
 * \n\
 * Request device information: \n\
 *   qd <node_name> \n\
 *   qd all \n\
 * \n\
 * Kill node: \n\
 *   k <node_name> <delay_ns> \n\
 * \n\
 * Ignore item: '-' \n\
 * \n\
 * Quit: \n\
 *   !\n\
 * Help: \n\
 *   ?\n\
 */\n");
}

int main(int argc, char* argv[])
{
    // Ctrl-c handler
    signal(SIGINT, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGTERM handler
    signal(SIGTERM, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGKILL handler
    signal(SIGKILL, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    rclcpp::init(argc, argv);
    auto controlNode = std::make_shared<rclcpp::Node>("devmanagecontrol_default");
    std::string serviceName = DEFAULT_SERVICE_NAME;
    rv2_interfaces::GetParam(controlNode, "devManageService", serviceName, serviceName, "devManageService: ", false);
    auto execNode = std::make_shared<rv2_interfaces::ExecNode>(controlNode);
    execNode->start();

    std::cout << "Service name: " << serviceName << std::endl;
    PrintHelp();

    while (!__globalExitFlag)
    {
        std::this_thread::sleep_for(100ms);
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);

        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = rv2_interfaces::split(inputStr, ", ");
        if (inputStrVec.size() == 1 && inputStrVec[0] == "!")
            __globalExitFlag = true;
        else if (inputStrVec.size() == 1 && inputStrVec[0] == "?")
            PrintHelp();

        if (inputStrVec[0] == "g")
        {
            auto req = std::make_shared<rv2_interfaces::srv::NodeAddrReg::Request>();

            if (inputStrVec.size() >= 2)
                req->node_addr.node_name = inputStrVec[1] == "-" ? GenDevInfoContent(NODENAME) : inputStrVec[1];
            else
                req->node_addr.node_name = GenDevInfoContent(NODENAME);

            if (inputStrVec.size() >= 3)
                req->node_addr.hostname = inputStrVec[2] == "-" ? GenDevInfoContent(HOSTNAME) : inputStrVec[2];
            else
                req->node_addr.hostname = GenDevInfoContent(HOSTNAME);

            if (inputStrVec.size() >= 4)
                req->node_addr.ipv4_addr = inputStrVec[3] == "-" ? GenDevInfoContent(IPV4) : inputStrVec[3];
            else
                req->node_addr.ipv4_addr = GenDevInfoContent(IPV4);

            if (inputStrVec.size() >= 5)
                req->node_addr.mac_addr = inputStrVec[4] == "-" ? GenDevInfoContent(MAC) : inputStrVec[4];
            else
                req->node_addr.mac_addr = GenDevInfoContent(MAC);

            std::cout << "Reg: " << req->node_addr.node_name << "[" << req->node_addr.hostname << "]\t" << req->node_addr.ipv4_addr << "/" << req->node_addr.mac_addr << std::endl;
            auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::NodeAddrReg>(controlNode, DevManageServer_NodeAddrRegSrvName(serviceName), req, 500ms);
            if (res)
                std::cout << "Request: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
        }
        else if (inputStrVec[0] == "k" && inputStrVec.size() > 2)
        {
            try
            {
                auto req = std::make_shared<rv2_interfaces::srv::DevManageServer::Request>();
                req->request.server_action = rv2_interfaces::msg::DevManageServerStatus::SERVER_ACTION_KILL_NODE;
                req->request.server_kill_node_name = inputStrVec[1];
                req->request.server_kill_node_exit_code = 1;
                req->request.server_kill_node_delay_ns = std::stoi(inputStrVec[2]);

                auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::DevManageServer>(controlNode, DevManageServer_DevManageServerSrvName(serviceName), req, 500ms);
                if (res)
                {
                    std::cout << "Request: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
                }
            }
            catch(...)
            {
                std::cerr << "Invalid input" << std::endl;
            }
        }
        else if (inputStrVec[0] == "qn")
        {
            auto req = std::make_shared<rv2_interfaces::srv::NodeAddrReq::Request>();

            if (inputStrVec.size() > 1)
                req->node_addr.node_name = inputStrVec[1] == "-" ? "" : inputStrVec[1];
            if (inputStrVec.size() > 2)
                req->node_addr.hostname = inputStrVec[2] == "-" ? "" : inputStrVec[2];
            if (inputStrVec.size() > 3)
                req->node_addr.ipv4_addr = inputStrVec[3] == "-" ? "" : inputStrVec[3];
            if (inputStrVec.size() > 4)
                req->node_addr.mac_addr = inputStrVec[4] == "-" ? "" : inputStrVec[4];

            auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::NodeAddrReq>(controlNode, DevManageServer_NodeAddrReqSrvName(serviceName), req, 500ms);
            if (res)
            {
                std::cout << "Request: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
                if (res->response == rv2_interfaces::ServiceResponseStatus::SRV_RES_SUCCESS)
                {
                    std::cout << std::left << std::setw(5) << "NO." << std::setw(28) << "NODE" << std::setw(20) << "HOST" << std::setw(16) << "IPv4" << "MAC" << std::endl;
                    std::cout << "--------------------------------------------------------------------------------------" << std::endl;
                    for (size_t i = 0; i < res->node_addr_vec.size(); i++)
                    {
                        std::cout << std::left << std::setw(5) << i + 1 << rv2_interfaces::GetNodeAddrStr(res->node_addr_vec[i]) << std::endl;
                    }
                }
            }
        }
        else if (inputStrVec[0] == "qd" && inputStrVec.size() > 1)
        {
            auto req = std::make_shared<rv2_interfaces::srv::DevInfoReq::Request>();
            req->node_addr.node_name = inputStrVec[1];

            auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::DevInfoReq>(controlNode, DevManageServer_DevInfoReqSrvName(serviceName), req, 500ms);
            if (res)
            {
                std::cout << "Request: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
                rv2_interfaces::HierarchicalPrint hprint;
                for (const auto& devInfo : res->dev_info_vec)
                {
                    rv2_interfaces::AddHierarchicalPrint(hprint, 0, devInfo);
                }
                hprint.print();
                hprint.clear();
            }
        }
    }

    execNode.reset();

    rclcpp::shutdown();
}

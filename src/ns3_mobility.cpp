//
// Created by harshal on 7/8/19.
//

#include "ns3/core-module.h"
//#include "ns3/mobility-module.h"
//#include "ns3/wifi-module.h"
//#include "ns3/internet-module.h"
//#include "ns3/fd-net-device-module.h"
//#include <ros/ros.h>


using namespace ns3;

int main (int argc, char *argv[]){

    NS_LOG_COMPONENT_DEFINE ("This is my first try");

    NS_LOG_UNCOND ("This is my first try");

    std::printf("this is my first try");

    Time::SetResolution(Time::NS);


    // NodeContainer nodes;
    // nodes.Create (2);

    Simulator::Run ();
    Simulator::Destroy ();
}
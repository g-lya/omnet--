#include "EVLC.h"

#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <algorithm>
#include <string.h>
#include <stdio.h>

Define_Module(PSEmergency);

void PSEmergency::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;

        // read parameters
//        numberOfRoads = par("numberOfRoads");
        numberOfRoads = 2;
        onDuty = par("onDuty").boolValue();
        currentSubscribedServiceId = -1;
    }
    else if (stage == 1) {
        //Initializing members that require initialized other modules goes here

    }
}

void PSEmergency::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void PSEmergency::onBSM(BasicSafetyMessage* bsm) {
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here

}

void PSEmergency::onWSM(WaveShortMessage* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

    findHost()->getDisplayString().updateWith("r=60,green");

    if (onDuty == false){ // a vehicle which is not on Public Safety duty
        if (onRoute(wsm->getWsmData())) {  //if they are in the same lane
            StopFollowVeh();
            std::cout << "Noted. I am braking!" << std::endl;
            // TODO
        }
    }

}

void PSEmergency::onWSA(WaveServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples


    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Emergency Warning!");
        }
    }
}

void PSEmergency::handleSelfMsg(cMessage* msg) {
    BaseWaveApplLayer::handleSelfMsg(msg);
    //this method is for self messages (mostly timers)
    //it is important to call the BaseWaveApplLayer function for BSM and WSM transmission

    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
        //send this message on the service channel until the counter is 3 or higher.
        //this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 3) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
           scheduleAt(simTime()+1, wsm);

        }
    }
    else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void PSEmergency::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    if (onDuty){
        findHost()->getDisplayString().updateWith("r=60,red");

        WaveShortMessage* wsm = new WaveShortMessage();

        populateWSM(wsm);

        std::cout<< "I am the leading vehicle. ID:" << wsm->getSenderAddress() <<
                ". I have to brake! This is my route: " << getCourse(numberOfRoads) << std::endl;

        std::cout << "Vehicle. ID:[" << this->myId << "] curSpeed.x " << this->curSpeed.x << std::endl;
        const char * speed = "curSpeed.x";

        wsm->setWsmData(speed);
        wsm->setWsmData(getCourse(numberOfRoads).c_str());
        //      wsm->setPsid(12);
        //      wsm->setPsc("Give way! Public Safety vehicle is on Duty!");
        //      wsm->setSecurityType(); TODO Implementar segurança

        if (dataOnSch) {
            startService(Channels::SCH2, 12, "Emergency Warning");
            //started service and server advertising, schedule message to self to send later
            scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
        }
        else {
            //send right away on CCH, because channel switching is disabled
            sendDown(wsm);
        }
    }

}

std::string PSEmergency::formatRoad(std::string road) {
    return "[" + road + "]";
}

std::string PSEmergency::formatLane(int lane) {
    return "(" + std::to_string(lane) + ")";
}

std::string PSEmergency::getCourse(int maxNumberOfRoads) {

    std::list<std::string> route = traciVehicle->getPlannedRoadIds();
    std::string roadInTransit = formatRoad(traciVehicle->getRoadId());
    std::string formattedCourse = "";

    int addedRoads = 0;
    bool addInMessage = false;

    //
    for (std::list<std::string>::iterator roadId = route.begin(); roadId != route.end(); ++roadId) {
        std::string road = formatRoad(*roadId);
       if (road != roadInTransit)
            addInMessage = true;

        if (addInMessage) {
            if (addedRoads < maxNumberOfRoads) {
                formattedCourse = formattedCourse + road;
                addedRoads++;
            } else {
                break;
            }
        }
    }

    formattedCourse = formatLane(traciVehicle->getLaneIndex())
            + formattedCourse;

    return formattedCourse;
}

bool PSEmergency::onRoute(std::string course) {

    std::string roadInTransit = formatRoad(traciVehicle->getRoadId());
    std::string laneInTransit = formatLane(traciVehicle->getLaneIndex());

    if (course.find(roadInTransit) != std::string::npos)
        if (course.find(laneInTransit) != std::string::npos)
            return true; //on route

    return false; //not on route
}

void PSEmergency::StopFollowVeh() {

//    Coord Position0 = curPosition;
    double Position0 = curPosition.x;
    double Position1 = mobility->getPositionAt(simTime()).x;

    FILE *fp = fopen("desiredspeed.txt", "a");
    fprintf(fp, "%d/n", curSpeed.x);
    fclose(fp);

    FILE *fp0 = fopen("position0_all.txt", "a");
    fprintf(fp0, "%d/n", Position0);
    fclose(fp0);

    FILE *fp1 = fopen("position1_all.txt", "a");
    fprintf(fp1, "%d/n", Position1);
    fclose(fp1);

    FILE *fp3;
    char buffer[1024];
    size_t t;

    FILE *trriger;
    size_t a;


    fp3 = fopen("deceleration.txt", "r");
       t = fread(buffer, sizeof(char), 1024, fp3);
       fclose(fp3);

    trriger = fopen("trriger.txt", "r");
       a = fread(buffer, sizeof(char), 1024, trriger);

    int c = 0;

    if(c==1)
      traciVehicle->setSpeed(30);
    else
      traciVehicle->setSpeed(10);

    traciVehicle->slowDown(0, t);


}



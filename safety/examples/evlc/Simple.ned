package examples.evlc;

import org.car2x.veins.modules.mobility.traci.TraCIScenarioManagerForker;
import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network Simple extends Scenario
{
    parameters:
        int numberOfRSUs = default(1); //whether to print debug output
    submodules:
        rsu[numberOfRSUs]: RSU {
            @display("p=150,140;b=10,10,oval");
        }

}


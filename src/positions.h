#include <map>
#include <vector>


std::vector<double> getPosition(std::string position){

        ROS_INFO(">>>> goal position [%s] <<<<", position.c_str());
        
        // Map to store coordinates - Key = description as string, Value = vector with double value of the joint positions
        std::map<std::string, std::vector<double>> positions;

        // Position above printer
        positions["initial position"] =
                {
                +1.498139,		 // Joint 1 
                -0.520210,		 // Joint 2 
                +0.097301,		 // Joint 3 
                -2.814416,		 // Joint 4 
                +0.052662,		 // Joint 5 
                +2.351891,		 // Joint 6 
                +0.801563		 // Joint 7 
                };

        // Position above printer
        positions["near printer"] =
                {
                +2.496836,		 // Joint 1 
                +0.772564,		 // Joint 2 
                +1.106343,		 // Joint 3 
                -1.478179,		 // Joint 4 
                -0.645623,		 // Joint 5 
                +1.874790,		 // Joint 6 
                -0.199039		 // Joint 7 
                };

        // Position at printer
        positions["printer"] =
                {
                +2.605499,		// Joint 1
                +1.441675,		// Joint 2
                +1.236548,		// Joint 3
                -1.397383,		// Joint 4
                -0.777287,		// Joint 5
                +2.323873,		// Joint 6
                -0.287551		// Joint 7
                };

        // Position 50cm above conveyor belt
        positions["near conveyor belt"] =
                {
                +0.844795,		 // Joint 1 
                -0.384217,		 // Joint 2 
                +1.890267,		 // Joint 3 
                -1.501299,		 // Joint 4 
                +0.146468,		 // Joint 5 
                +1.830871,		 // Joint 6 
                +2.016475		 // Joint 7 
                };

        // Position at conveyor belt
        positions["conveyor belt"] =
                {
                +0.972066,		// Joint 0
                -1.419312,		// Joint 1
                +1.952559,		// Joint 2
                -1.774845,		// Joint 3
                +0.277226,		// Joint 4
                +2.372050,		// Joint 5
                +2.414976		// Joint 6
                };

        // Position at storage
        positions["storage"] =
                {
                +2.031532,		 // Joint 1 
                +1.054223,		 // Joint 2 
                +2.168870,		 // Joint 3 
                -2.671940,		 // Joint 4 
                -0.792631,		 // Joint 5 
                +3.065037,		 // Joint 6 
                +0.802607		 // Joint 7 
                };

        // Position near storage place 1
        positions["near storage place 1"] =
                {
                +1.900263,		 // Joint 1 
                +0.559978,		 // Joint 2 
                +2.802849,		 // Joint 3 
                -2.067452,		 // Joint 4 
                -0.311117,		 // Joint 5 
                +2.489097,		 // Joint 6 
                +0.959915		 // Joint 7 
                };

        // Position near storage place 1
        positions["storage place 1"] =
                {
                +1.902461,		 // Joint 1 
                +0.410919,		 // Joint 2 
                +2.765295,		 // Joint 3 
                -2.040703,		 // Joint 4 
                -0.174810,		 // Joint 5 
                +2.600436,		 // Joint 6 
                +0.832778		 // Joint 7 
                };

        // Position near storage place 2
        positions["near storage place 2"] =
                {
                +1.853758,		 // Joint 1 
                +0.514412,		 // Joint 2 
                +2.624540,		 // Joint 3 
                -1.938238,		 // Joint 4 
                -0.033951,		 // Joint 5 
                +2.403973,		 // Joint 6 
                +0.527461		 // Joint 7 
                };

        // Position near storage place 2
        positions["storage place 2"] =
                {
                +1.841644,		 // Joint 1 
                +0.319584,		 // Joint 2 
                +2.607986,		 // Joint 3 
                -1.886534,		 // Joint 4 
                +0.033933,		 // Joint 5 
                +2.464868,		 // Joint 6 
                +0.527462		 // Joint 7 
                };

        // Position near storage place 3
        positions["near storage place 3"] =
                {
                +1.793602,		 // Joint 1 
                +0.293711,		 // Joint 2 
                +2.425398,		 // Joint 3 
                -1.710999,		 // Joint 4 
                +0.358704,		 // Joint 5 
                +2.326216,		 // Joint 6 
                +0.096573		 // Joint 7 
                };

        // Position near storage place 3
        positions["storage place 3"] =
                {
                +1.785067,		 // Joint 1 
                +0.111577,		 // Joint 2 
                +2.395085,		 // Joint 3 
                -1.661979,		 // Joint 4 
                +0.492932,		 // Joint 5 
                +2.331837,		 // Joint 6 
                +0.039065		 // Joint 7 
                };

        // Position near storage place 4
        positions["near storage place 4"] =
                {
                +1.704027,		 // Joint 1 
                +0.584230,		 // Joint 2 
                +2.875369,		 // Joint 3 
                -2.515814,		 // Joint 4 
                -0.455071,		 // Joint 5 
                +2.950280,		 // Joint 6 
                +1.117243		 // Joint 7 
                };

        // Position near storage place 4
        positions["storage place 4"] =
                {
                +1.725847,		 // Joint 1 
                +0.306364,		 // Joint 2 
                +2.863083,		 // Joint 3 
                -2.341458,		 // Joint 4 
                -0.448853,		 // Joint 5 
                +2.911509,		 // Joint 6 
                +1.117706		 // Joint 7 
                };

        // Position near storage place 5
        positions["near storage place 5"] =
                {
                +1.899778,		 // Joint 1 
                +0.622851,		 // Joint 2 
                +2.525775,		 // Joint 3 
                -2.343862,		 // Joint 4 
                -0.234449,		 // Joint 5 
                +2.633637,		 // Joint 6 
                +0.642411		 // Joint 7 
                };

        // Position near storage place 5
        positions["storage place 5"] =
                {
                +2.044198,		 // Joint 1 
                +0.296671,		 // Joint 2 
                +2.394162,		 // Joint 3 
                -2.164684,		 // Joint 4 
                +0.020413,		 // Joint 5 
                +2.641030,		 // Joint 6 
                +0.481424		 // Joint 7 
                };


        // Position near storage place 6
        positions["near storage place 6"] =
                {
                +2.439258,		 // Joint 1 
                +0.950777,		 // Joint 2 
                +1.958296,		 // Joint 3 
                -2.145058,		 // Joint 4 
                -0.788928,		 // Joint 5 
                +2.610972,		 // Joint 6 
                +0.723250		 // Joint 7 
                };

        // Position near storage place 6
        positions["storage place 6"] =
                {
                +2.660545,		 // Joint 1 
                +0.911755,		 // Joint 2 
                +1.782103,		 // Joint 3 
                -2.025104,		 // Joint 4 
                -0.883853,		 // Joint 5 
                +2.615542,		 // Joint 6 
                +0.767611		 // Joint 7 
                };

        // Position near storage place 7
        positions["near storage place 7"] =
                {
                +1.629001,		 // Joint 1 
                +0.332123,		 // Joint 2 
                +2.875330,		 // Joint 3 
                -2.677662,		 // Joint 4 
                -0.396351,		 // Joint 5 
                +3.259462,		 // Joint 6 
                +1.011538		 // Joint 7 
                };

        // Position near storage place 7
        positions["storage place 7"] =
                {
                +1.644874,		 // Joint 1 
                +0.044384,		 // Joint 2 
                +2.876314,		 // Joint 3 
                -2.513478,		 // Joint 4 
                -0.377289,		 // Joint 5 
                +3.348445,		 // Joint 6 
                +1.079716		 // Joint 7 
                };

        // Position near storage place 8
        positions["near storage place 8"] =
                {
                +1.405430,		 // Joint 1 
                +0.055462,		 // Joint 2 
                +2.618529,		 // Joint 3 
                -2.489355,		 // Joint 4 
                -1.104544,		 // Joint 5 
                +3.744352,		 // Joint 6 
                +1.480782		 // Joint 7 
                };

        // Position near storage place 8
        positions["storage place 8"] =
                {
                +1.432385,		 // Joint 1 
                -0.160770,		 // Joint 2 
                +2.681322,		 // Joint 3 
                -2.348030,		 // Joint 4 
                -0.984436,		 // Joint 5 
                +3.739133,		 // Joint 6 
                +1.481079		 // Joint 7 
                };


        // Position near storage place 9
        positions["near storage place 9"] =
                {
                +1.251620,		 // Joint 1 
                +0.139746,		 // Joint 2 
                +2.852862,		 // Joint 3 
                -2.300409,		 // Joint 4 
                +1.745586,		 // Joint 5 
                +2.658023,		 // Joint 6 
                -1.256127		 // Joint 7 
                };

        // Position near storage place 9
        positions["storage place 9"] =
                {
                +1.284026,		 // Joint 1 
                -0.101459,		 // Joint 2 
                +2.863415,		 // Joint 3 
                -2.128719,		 // Joint 4 
                +1.742942,		 // Joint 5 
                +2.609693,		 // Joint 6 
                -1.250185		 // Joint 7 
                };

        // Position above desk - pointing to camera
        positions["camera"] =
                {
                +0.861129,		 // Joint 1 
                +0.835717,		 // Joint 2 
                +0.199601,		 // Joint 3 
                -0.577428,		 // Joint 4 
                -0.169529,		 // Joint 5 
                +3.432604,		 // Joint 6 
                +0.994774		 // Joint 7 
                };

        // Position above desk - pointing to camera
        positions["desk right"] =
                {
                +0.428084,		// Joint 1
                +1.016777,		// Joint 2
                +0.352650,		// Joint 3
                -1.439887,		// Joint 4
                -0.625543,		// Joint 5
                +2.445297,		// Joint 6
                +1.911189		// Joint 7
                };

        // Position above desk - pointing to camera
        positions["near desk right"] =
                {
                +0.424486,		// Joint 1
                +0.827581,		// Joint 2
                +0.357930,		// Joint 3
                -1.493054,		// Joint 4
                -0.566994,		// Joint 5
                +2.321353,		// Joint 6
                +1.885754		// Joint 7
                };

        // Position above desk - pointing to camera
        positions["desk left"] =
                {
                -1.144081,		// Joint 1
                +1.164832,		// Joint 2
                +0.432686,		// Joint 3
                -1.217856,		// Joint 4
                -0.331640,		// Joint 5
                +2.463407,		// Joint 6
                +0.085543		// Joint 7
                };

        // Position above desk - pointing to camera
        positions["near desk left"] =
                {
                -0.986104,		// Joint 1
                +0.979894,		// Joint 2
                +0.253666,		// Joint 3
                -1.205803,		// Joint 4
                -0.258403,		// Joint 5
                +2.240249,		// Joint 6
                +0.130150		// Joint 7
                };

        return positions[position];
}
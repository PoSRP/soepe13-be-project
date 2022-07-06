# BE Project

## Description

## Phases
### Phase One
Developing a ROS2 Node to server as an EtherCAT Master using the SOEM library. The Node would be responsible for the following:  
  - Control of the EtherCAT FSM.
  - Control of slave CiA-402 FSMs.
  - Publishing messages from the EtherCAT network to the ROS2 network.
  - Providing Actions for controlling parameters on the EtherCAT network.
  - Providing Services for viewing parameters on the EtherCAT network.

Developing an RQt integrated explorer for the parameters presented by the EtherCAT Master Node. It would provide the following:
  - Ability to select specific slave parameters and collate them in a collected view.
  - Ability to view and set parameters through the collected view.

Developing an RQt integrated control panel for the EtherCAT server Node. It would provide the following:
  - Ability to send each specific state changing command to the network or specific slave.
  - Ability to select network or specific slave state and have it automatically change.
  - Ability to view the state of each part of the EtherCAT network.

#### *Intestigation*
  - How to achieve easy ROS2 integration of various EtherCAT drives compliant to the CiA-402 communication profile.  
  - How to fascilitate rapid updating of EtherCAT network parameters through a ROS2 integrated interface.  

#### *Experimentation*
  - Connecting devices from different manufacturers.  
  - Setting and verifying network parameters through a basic interface.  

#### *Components*
  - ROS2 EtherCAT Server Node
  - RQt Network Parameter Explorer 
  - RQt Network and Slave FSM Controls

### Phase Two
Developing an interface for running predefined movement profiles. It would provide the following:
  - Planning of typical motion profiles (synchronous). 
  - Assignment of slave(s) to run during movement. 
  - Graphing of the expected versus achieved movement.  
  - Information about expected and achieved movements. 

Devising a simple CiA-402 drive compliance screening procedure.  
Developing a generalized quality parameter for evaluating movement profiles.  
Developing a generalized procedure for estimating setup friction within the CiA-402 profile.  

#### *Investigation*
  - How to quantify the quality of an achieved movement profile.  
  - How to generalize friction estimation using CiA-402.  
  - How to satisfy compliance testing within the EtherCAT CiA-402 standard. 

#### *Experimentation*
  - Examination using disturbed vs undisturbed drive movement.  
  - Comparison of friction values calculated using different methods.  
  - Running the developed compliance screening.  

#### Components
  - RQt Profile Movement Control Panel
  - RQt EtherCAT CiA-402 Compliance Tester

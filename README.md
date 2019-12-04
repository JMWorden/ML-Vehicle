# ML-Vehicle

## Primary Resources
- [Unity3D](https://unity.com/releases/2019-2)
- [ML-Agents](https://github.com/Unity-Technologies/ml-agents)
- [Simple Town - Cartoon Assets](https://assetstore.unity.com/packages/3d/environments/urban/simple-town-cartoon-assets-43500)

## Run Instructions

Execute 'Build/Vehicle_Project.exe'.

## Design Documentation

### Unity Project Scripts
- [VehiclePhysics.cs](https://github.com/JMWorden/ML-Vehicle/blob/master/VehicleProject/Vehicle_Project/Assets/Scripts/VehiclePhysics.cs): MonoBehaviour that determines some physics characteristics of the vehicle game object. This includes wheel colliders; motor, steering, and braking input; proximity sensors; GUI output; and frame updates. 
- [VehicleAgent.cs](https://github.com/JMWorden/ML-Vehicle/blob/master/VehicleProject/Vehicle_Project/Assets/Scripts/VehicleAgent.cs): Extends [Agent](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Design-Agents.md). Builds vector of observations to send to brain; applies brake, motor torque, and steering on the vehicle based on vector of actions recieved from brain; builds reward signal; resets agent (and goal) on done().
- [VehicleAcademy.cs](https://github.com/JMWorden/ML-Vehicle/blob/master/VehicleProject/Vehicle_Project/Assets/Scripts/VehicleAcademy.cs): Simply extends [Academy](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Design-Academy.md).
- [SpawnAreaManager.cs](https://github.com/JMWorden/ML-Vehicle/blob/master/VehicleProject/Vehicle_Project/Assets/Scripts/SpawnAreaManager.cs). Used to find safe, random points in the world model to spawn the vehicle and goal on AgentReset().

### Hyper Parameters for Vehicle NN (Summary)
- Policy: [PPO](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Training-PPO.md)
- Total training steps: 100,000,000 (1.0e8)
- Number of layers: 3
- Neurons per layer: 512

The actual configuration file laying out all hyper-parameters used during training can be found [here](https://github.com/JMWorden/ML-Vehicle/blob/master/ml-agents/config/trainer_config.yaml).

### Training Description
Agent was trained for 7d 14h 40m 43s. Up until around 2d 8h 51m 51s of training, the agent was completely incompetent. It seemed to reach peak performance at 4d 13h 31m 30s -- after that point, additional training yielded no substantial increases in performance (measured by average extrinsic reward). Below is a screen capture of the TensorBoard graphs at the end of training.

![Training Graphs](https://github.com/JMWorden/ML-Vehicle/blob/master/TensorBoard_training_screencap.jpg "TensorBoard Training Graphs")

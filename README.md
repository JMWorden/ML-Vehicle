# ML-Vehicle

## Primary Resources
- [Unity3D](https://unity.com/releases/2019-2)
- [ML-Agents](https://github.com/Unity-Technologies/ml-agents)
- [Simple Town - Cartoon Assets](https://assetstore.unity.com/packages/3d/environments/urban/simple-town-cartoon-assets-43500)

## Vehicle Agent

### Hyper Parameters for Vehicle NN (Summary)
- Policy: PPO
- Total training steps: 100,000,000 (1.0e8)
- Number of layers: 3
- Neurons per layer: 512

### Training Description

[Training Graphs]: https://github.com/JMWorden/ML-Vehicle/blob/master/TensorBoard_training_screencap.jpg "TensorBoard Training Graphs"

Agent was trained for 7d 14h 40m 43s. Up until around 2d 8h 51m 51s of training, the agent was completely incompetent. It seemed to reach peak performance at 4d 13h 31m 30s -- after that point, additional training yielded no substantial increases in performance (measured by average extrinsic reward).

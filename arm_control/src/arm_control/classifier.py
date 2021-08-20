import torch
import numpy as np
from arm_control.TSD_neural_network import TSD_Network

path_weights ="/home/laiy/gitrepos/myo_ws/src/arm_control/src/arm_control/model/best_state_0.pt"
# path_weights ="/home/laiy/gitrepos/msr_final/LongTermEMG_myo/TrainingsAndEvaluations/ForTrainingSessions/Weights_ros/TSD/participant_0/best_state_0.pt"
num_kernels=[200, 200, 200]                        # model layer size 
number_of_classes=5
feature_vector_input_length=252                     # size of one example 

model = TSD_Network(number_of_class=number_of_classes, num_neurons=num_kernels,
                            feature_vector_input_length=feature_vector_input_length)

best_state = torch.load(path_weights)
best_weights = best_state['state_dict']
model.load_state_dict(best_weights)

def predict(example):
    inputs  = torch.from_numpy(np.array([example], dtype=np.float32))
    model.eval()
    output = model(inputs)
    _, predicted = torch.max(output.data, 1)
    return predicted.numpy()[0]
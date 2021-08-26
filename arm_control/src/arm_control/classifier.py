"""
Includes a helper function for predicting right hand gesture using the long-term myo classifier. 
When this file is initialized, load a model from model/best_state_0.pt. This model is trained using one subject
and five selected gestures from the long-term myo dataset 
(https://github.com/Suguru55/Wearable_Sensor_Long-term_sEMG_Dataset). 

See https://github.com/aonai/long_term_myo_notes/blob/main/test_code/train_for_ros_myo.ipynb
and https://github.com/aonai/long_term_myo_notes/blob/main/test_code/train_for_ros_myo_one_subject.ipynb 
for notes on the trained model. 

See https://github.com/aonai/long_term_EMG_myo for how to train a models. model_utils.py, TSD_neural_network.py,
and util.py are copies of code in this repo. 
"""

import torch
import numpy as np
from arm_control.TSD_neural_network import TSD_Network

# model params
path_weights ="/home/laiy/gitrepos/myo_ws/src/arm_control/src/arm_control/model/best_state_0.pt"
num_kernels=[200, 200, 200]                       
number_of_classes=5
feature_vector_input_length=252                     

model = TSD_Network(number_of_class=number_of_classes, num_neurons=num_kernels,
                            feature_vector_input_length=feature_vector_input_length)

best_state = torch.load(path_weights)
best_weights = best_state['state_dict']
model.load_state_dict(best_weights)

def predict(example):
    """ Predict right-hand gesture 

        Args:
            example (list of double) - list of 8 EMG signals that are already formated (normalized,
                                        filtered, then applied TDS feature function)
        
        Returns:
            prediction (int) - predicted gesture index 
                                REST = 0
                                WF = 1      # wrist extension
                                WE = 2      # wrist flexion
                                RD = 3      # radial deviation
                                UD = 4      # ulnar deviation 
    """
    inputs  = torch.from_numpy(np.array([example], dtype=np.float32))
    model.eval()
    output = model(inputs)
    _, predicted = torch.max(output.data, 1)
    return predicted.numpy()[0]
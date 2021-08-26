#!/usr/bin/env python3
from scipy.signal.filter_design import normalize
from scipy.signal import butter, filtfilt
import numpy as np
from itertools import combinations


def getTSD(all_channels_data_in_window):
    """
    Original by RamiKhushaba
        https://github.com/RamiKhushaba/getTSDfeat
    
    Edited by UlysseCoteAllard
        https://github.com/Suguru55/Wearable_Sensor_Long-term_sEMG_Dataset

    Note: this uses window emg directly instead of extracting a window then apply TSD like in Rami's code. Calculation
    is mainly based on equations explained in [3].

    Note2: it looks like the window size needs to be at least 28 to have this functions working; otherwise, the calcualted 
            features will not be able to fill up all the combinations 
    
    %Implementation from Rami Khusaba adapted to our code
    % References
    % [1] A. Al-Timemy, R. N. Khushaba, G. Bugmann, and J. Escudero, "Improving the Performance Against Force Variation of EMG Controlled Multifunctional Upper-Limb Prostheses for Transradial Amputees",
    %     IEEE Transactions on Neural Systems and Rehabilitation Engineering, DOI: 10.1109/TNSRE.2015.2445634, 2015.
    % [2] R. N. Khushaba, Maen Takruri, Jaime Valls Miro, and Sarath Kodagoda, "Towards limb position invariant myoelectric pattern recognition using time-dependent spectral features",
    %     Neural Networks, vol. 55, pp. 42-58, 2014.

    [3] R. N. Khushaba, A. H. Al-Timemy, A. Al-Ani and A. Al-Jumaily, "A Framework of Temporal-Spatial Descriptors-Based 
        Feature Extraction for Improved Myoelectric Pattern Recognition," in IEEE Transactions on Neural Systems and
        Rehabilitation Engineering, vol. 25, no. 10, pp. 1821-1831, Oct. 2017, doi: 10.1109/TNSRE.2017.2687520.
    """
    # x should be a numpy array
    all_channels_data_in_window = np.swapaxes(np.array(all_channels_data_in_window), 1, 0)

    if len(all_channels_data_in_window.shape) == 1:
        all_channels_data_in_window = all_channels_data_in_window[:, np.newaxis]

    datasize = all_channels_data_in_window.shape[0]
    Nsignals = all_channels_data_in_window.shape[1]
    # print("all_channels_data_in_window  = ", all_channels_data_in_window.shape)

    # prepare indices of each 2 channels combinations
    # NCC = Number of channels to combine
    NCC = 2
    Indx = np.array(list(combinations(range(Nsignals), NCC)))   # (28,2)

    # allocate memory
    # define the number of features per channel
    NFPC = 7

    # Preallocate memory
    feat = np.zeros((Indx.shape[0] * NFPC + Nsignals * NFPC))

    # Step1.1: Extract between-channels features
    ebp = getTSDfeatures_for_one_representation(
        all_channels_data_in_window[:, Indx[:, 0]] - all_channels_data_in_window[:, Indx[:, 1]])
    efp = getTSDfeatures_for_one_representation(
        np.log(
            (all_channels_data_in_window[:, Indx[:, 0]] - all_channels_data_in_window[:, Indx[:, 1]]) ** 2 + np.spacing(
                1)) ** 2)
    # Step 1.2: Correlation analysis
    num = np.multiply(efp, ebp)
    den = np.sqrt(np.multiply(efp, efp)) + np.sqrt(np.multiply(ebp, ebp))
    # print("num = ", np.shape(num))
    # print("put into feat = ", np.shape(range(Indx.shape[0] * NFPC)))
    feat[range(Indx.shape[0] * NFPC)] = num / den

    # Step2.1: Extract within-channels features
    ebp = getTSDfeatures_for_one_representation(all_channels_data_in_window)
    efp = getTSDfeatures_for_one_representation(np.log((all_channels_data_in_window) ** 2 + np.spacing(1)) ** 2)
    # Step2.2: Correlation analysis
    num = np.multiply(efp, ebp)
    den = np.sqrt(np.multiply(efp, efp)) + np.sqrt(np.multiply(ebp, ebp))
    feat[np.max(range(Indx.shape[0] * NFPC)) + 1:] = num / den
    return feat

def getTSDfeatures_for_one_representation(vector):
    """
    Original by RamiKhushaba
        https://github.com/RamiKhushaba/getTSDfeat
    
    Edited by UlysseCoteAllard
        https://github.com/Suguru55/Wearable_Sensor_Long-term_sEMG_Dataset

    Refer to Equations in [3]

    % Implementation from Rami Khusaba
    % Time-domain power spectral moments (TD-PSD)
    % Using Fourier relations between time domina and frequency domain to
    % extract power spectral moments dircetly from time domain.
    %
    % Modifications
    % 17/11/2013  RK: Spectral moments first created.
    % 02/03/2014  AT: I added 1 to the function name to differentiate it from other versions from Rami
    % 01/02/2016  RK: Modifed this code intosomewhat deep structure
    %
    % References
    % [1] A. Al-Timemy, R. N. Khushaba, G. Bugmann, and J. Escudero, "Improving the Performance Against Force Variation of EMG Controlled Multifunctional Upper-Limb Prostheses for Transradial Amputees",
    %     IEEE Transactions on Neural Systems and Rehabilitation Engineering, DOI: 10.1109/TNSRE.2015.2445634, 2015.
    % [2] R. N. Khushaba, Maen Takruri, Jaime Valls Miro, and Sarath Kodagoda, "Towards limb position invariant myoelectric pattern recognition using time-dependent spectral features",
    %     Neural Networks, vol. 55, pp. 42-58, 2014.

    [3] R. N. Khushaba, A. H. Al-Timemy, A. Al-Ani and A. Al-Jumaily, "A Framework of Temporal-Spatial Descriptors-Based 
    Feature Extraction for Improved Myoelectric Pattern Recognition," in IEEE Transactions on Neural Systems and
    Rehabilitation Engineering, vol. 25, no. 10, pp. 1821-1831, Oct. 2017, doi: 10.1109/TNSRE.2017.2687520.
    """

    # Get the size of the input signal
    samples, channels = vector.shape

    if channels > samples:
        vector = np.transpose(vector)
        samples, channels = channels, samples

    # Root squared zero order moment normalized  
    # Equation (2) and (6) with lambda = 0.1
    m0 = np.sqrt(np.nansum(vector ** 2, axis=0))[:, np.newaxis]
    m0 = m0 ** .1 / .1

    # Prepare derivatives for higher order moments
    # Euqation (3) and (6)      
    d1 = np.diff(np.concatenate([np.zeros((1, channels)), vector], axis=0), n=1, axis=0)
    d2 = np.diff(np.concatenate([np.zeros((1, channels)), d1], axis=0), n=1, axis=0)

    # Root squared 2nd and 4th order moments normalized
    # Equation (4) and (6)
    m2 = (np.sqrt(np.nansum(d1 ** 2, axis=0)) / (samples - 1))[:, np.newaxis]
    m2 = m2 ** .1 / .1

    # Equation (5) and (6)
    m4 = (np.sqrt(np.nansum(d2 ** 2, axis=0)) / (samples - 1))[:, np.newaxis]
    m4 = m4 ** .1 / .1

    # Sparseness
    # Euqation (8)
    sparsi = m0 / np.sqrt(np.abs(np.multiply((m0 - m2) ** 2, (m0 - m4) ** 2)))

    # Irregularity Factor
    # Equation (9)
    IRF = m2 / np.sqrt(np.multiply(m0, m4))

    # Coefficient of Variation
    # Equation (10)
    tmp = np.nanmean(vector, axis=0)
    if 0 in tmp:   # avoid divison by zero case 
        tmp[tmp==0] = 1e-10
    COV = (np.nanstd(vector, axis=0, ddof=1) / tmp)[:, np.newaxis]

    # Teager-Kaiser energy operator
    # Equation (11)
    TEA = np.nansum(d1 ** 2 - np.multiply(vector[0:samples, :], d2), axis=0)[:, np.newaxis]

    # All features together
    # Maybe similar to Equation (11)
    STDD = np.nanstd(m0, axis=0, ddof=1)[:, np.newaxis]

    if channels > 2:
        Feat = np.concatenate((m0 / STDD, (m0 - m2) / STDD, (m0 - m4) / STDD, sparsi, IRF, COV, TEA), axis=0)
    else:
        Feat = np.concatenate((m0, m0 - m2, m0 - m4, sparsi, IRF, COV, TEA), axis=0)

    Feat = np.log(np.abs(Feat)).flatten()

    return Feat

def butter_highpass(highcut=15.0, fs=200.0, order=5):
    """ 
    High-pass butterworth filter at 15Hz
    https://stackoverflow.com/questions/39032325/python-high-pass-filter 
    https://scipy-cookbook.readthedocs.io/items/ButterworthBandpass.html
    """
    nyq = 0.5 * fs
    high = highcut / nyq
    b, a = butter(order, high, btype='high')
    return b, a

def butter_highpass_filter(data, highcut=15.0, fs=200.0, order=5):
    """ 
    High-pass butterworth filter at 15Hz
    https://stackoverflow.com/questions/39032325/python-high-pass-filter 
    https://scipy-cookbook.readthedocs.io/items/ButterworthBandpass.html
    """
    # print("fs = ", fs)
    b, a = butter_highpass(highcut, fs, order=order)
    y = filtfilt(b, a, data, axis=0)
    return y

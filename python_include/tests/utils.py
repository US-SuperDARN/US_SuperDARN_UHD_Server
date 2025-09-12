

import numpy as np


max_int16_value = np.iinfo(np.int16).max # +32767
min_int16_value = np.iinfo(np.int16).min # -32768

def complex_float_to_int16(complex_float_samples, name_of_samples):
    real_mat = np.real(complex_float_samples)
    imag_mat = np.imag(complex_float_samples)
    abs_max_value = max(abs(real_mat).max(),  abs(imag_mat).max())
#   RHM.logger.info("Abs max_value is {} (int16_max= {}, max_value / int16_max = {} ) ".format(abs_max_value, max_int16_value, abs_max_value / max_int16_value ))

    # check for clipping
    if (real_mat > max_int16_value).any() or (real_mat < min_int16_value).any() or (imag_mat > max_int16_value).any() or (imag_mat < min_int16_value).any():
 #      RHM.logger.error("Overflow error while casting beamformed rx samples to complex int16s.")
        OverflowError("converting float to int16: overflow error in casting {} to complex int".format(name_of_samples))
        real_mat = np.clip(real_mat, min_int16_value, max_int16_value)
        imag_mat = np.clip(imag_mat, min_int16_value, max_int16_value)

    complexInt32_pack_mat = (np.uint32(np.int16(real_mat)) << 16) + np.uint16(imag_mat)
    output_int16_samples =  complexInt32_pack_mat.tolist()
    return output_int16_samples
                                                                             

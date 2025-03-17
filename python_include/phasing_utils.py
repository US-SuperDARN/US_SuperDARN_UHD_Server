# phasing matrix utilities
# for calculating phase shifts and other array stuff..

import numpy as np

C = 3e8
verbose = False

# returns a complex number from a phase in radians
def rad_to_rect(rad):
    return np.exp(-1j * rad)

# nbeams - total number of beams
# beamnum - beam number to calculate azm
# beam sep - beam separation in degrees
# returns beam azm angle in radians
def calc_beam_azm_rad(nbeams, beamnum, beam_sep):
    # calculate beamforming shift..
    center_beam = (nbeams - 1.0) / 2.

    # calculate beam azimuth, in radians
  #  bmazm = np.deg2rad(90 + (beamnum - center_beam) * beam_sep)
    bmazm = np.deg2rad( (beamnum - center_beam) * beam_sep)
    if verbose:
        print("nBeams: {}, beamNum: {}, beam_sep: {}".format(nbeams, beamnum, beam_sep))
        print("  beam = {} degree".format(bmazm / np.pi*180))
    return bmazm

# calculates the phase increment between antennas
# to produce a mainlobe steering of bmazm at tfreq 
# tfreq in hz, bmazm in radians
def calc_phase_increment(bmazm, tfreq, x_spacing):
    # translate to phase increment
    wavelength = C / tfreq
    pshift = (2 * np.pi * x_spacing * np.sin(bmazm)) / wavelength
    if verbose:
        print("tfreq: {} x_spacing: {}\n pshift: {} degree".format(tfreq, x_spacing, pshift*180/np.pi))  
    return pshift 

# calculates a complex number for a phase shift
def calc_beamforming_phase_rect(antenna, pshift):
    beamforming_phase = rad_to_rect(antenna * pshift)
    return beamforming_phase

def beamform_uhd_samples(samples, phasing_matrix, n_samples, antennas, pack_uint32):
    beamformed_samples = np.ones(n_samples, dtype=np.complex_)

    for i in range(int(n_samples)):
        itemp = 0
        qtemp = 0

        for aidx in range(len(antennas)):
            itemp += np.real(samples[aidx][i]) * np.real(phasing_matrix[aidx]) - \
                np.imag(samples[aidx][i]) * np.imag(phasing_matrix[aidx])
            qtemp += np.real(samples[aidx][i]) * np.imag(phasing_matrix[aidx]) + \
                np.imag(samples[aidx][i]) * np.real(phasing_matrix[aidx])
        if pack_uint32:
            beamformed_samples[i] = complex_int32_pack(itemp, qtemp)
        else:
            beamformed_samples[i] = itemp + 1j * qtemp

    return beamformed_samples


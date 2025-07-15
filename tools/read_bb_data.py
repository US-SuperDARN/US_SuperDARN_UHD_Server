import datetime
import os
import bz2
import numpy as np

# This code basically was written by Dr. Bill Bristow! 
# I copied and slightly modified this to suit VT requirements!
# -- Bharat Kunduri

class BBData(object):
    """
    Read baseband data!
    """
    def __init__(self, date, radar, base_data_dir="/sd-data/", rnum=1, cnum="a"):
        self.date = date
        self.radar = radar
        self.curr_idx = 0
        # This is specific to VT file storage system
        self.filename = base_data_dir + str(date.year) + "/" +\
              "image_samples/bb_data/" + self.radar + "/" +\
              self.date.strftime("%Y%m%d%H%M") + "." + str(rnum) + ".iraw." + cnum
        if os.path.isfile(self.filename):
            print("File %s found, now reading data!" % (self.filename))
            self.data_records = self.read_raw_file(is_bz2=False)
        else:
            if os.path.isfile(self.filename + ".bz2"):
                self.filename += ".bz2"
                print("Compressed file %s found, now decompressing and reading!" % (self.filename))
                self.data_records = self.read_raw_file(is_bz2=True)
            else:
                print("File %s doesn't exist" % (self.filename))
                self.data_records = None
    
    def next_data(self,data):
        # global self.curr_idx
        self.curr_idx+=1
        return(data[self.curr_idx])
    

    def read_raw_file(self,is_bz2=False):
        #  read all data as int32
        if is_bz2:
            # with bz2.open(self.filename, "rb") as fp:
            #     bb_data = fp.read()
            dfile = bz2.BZ2File(self.filename).read()
            data = np.frombuffer(dfile, dtype=np.uint32)
        else:
            raw_file = open(self.filename , "rb")
            data = np.fromfile(raw_file, dtype=np.uint32)
            raw_file.close()

        
        all_periods = []
        
        while (self.curr_idx < len(data)):
            period_dict = {}
            period_dict['version'] = data[self.curr_idx]
            # print(data[self.curr_idx])
            if period_dict['version'] != 3:
                print("Error: only Version 3 exports are supported!")
                return 0
            # period_dict['stid'] = self.next_data(data)
            # self.curr_idx+=1
            # return(data[self.curr_idx])
            # period_dict['channel'] = self.next_data(data)
            period_dict['year'] = self.next_data(data)
            period_dict['month'] = self.next_data(data)
            period_dict['day'] = self.next_data(data)
            period_dict['hour'] = self.next_data(data)
            period_dict['minute'] = self.next_data(data)
            period_dict['second'] = self.next_data(data)
            period_dict['microsecond'] = self.next_data(data)
            period_dict['nrang'] = self.next_data(data)
            period_dict['mpinc'] = self.next_data(data)
            period_dict['smsep'] = self.next_data(data)
            period_dict['lagfr'] = self.next_data(data)
            period_dict['pulseLength'] = self.next_data(data)
            period_dict['beam'] = self.next_data(data)
        
            period_dict['rfreq'] = self.next_data(data)
            period_dict['mppul'] = self.next_data(data)
            self.curr_idx+=1
            period_dict['ppat'] = data[self.curr_idx:self.curr_idx+period_dict['mppul']]
            self.curr_idx = self.curr_idx+period_dict['mppul']
        
            period_dict['nbaud'] = data[self.curr_idx]
            self.curr_idx+=1
            period_dict['pcode'] = data[self.curr_idx:self.curr_idx+period_dict['nbaud']+1]
            self.curr_idx += period_dict['nbaud']
            
            period_dict['nSamples'] = data[self.curr_idx]
            period_dict['nSeq'] = self.next_data(data)
            period_dict['nAntennas'] = self.next_data(data)
            # print(period_dict)
            # asd
            self.curr_idx+=1
            period_dict['antenna_list']= data[self.curr_idx:self.curr_idx+period_dict['nAntennas']]
            self.curr_idx += period_dict['nAntennas']
            # print("  Integration period: {} with {} sequences".format(len(all_periods)+1, period_dict['nSeq'] ))
            seq_list = []
            for iSeq in range(period_dict["nSeq"]):
                seq_dict = {}
                seq_dict["sequence_no_in_period"] = iSeq
                seq_dict["seq_start_time_sec"] = data[self.curr_idx]
                seq_dict["seq_start_time_usec"] = self.next_data(data)
                samples = []
                nSamples = period_dict['nSamples'] *2 # because we read as uint32 but one sample is complex64 (or two float32)
                self.curr_idx+=1
                for iAntenna in range(period_dict['nAntennas']):
                    packed_data = data[self.curr_idx+iAntenna*nSamples:self.curr_idx+nSamples*(iAntenna+1) ]
                    packed_data.dtype = "complex64"
                    samples.append( packed_data )
                    #   samples.append( np.int16(packed_data >> 16) + 1j* np.int16(packed_data % 2**16))
        
                seq_dict["samples"] = samples
                seq_list.append(seq_dict)
                self.curr_idx += nSamples*period_dict['nAntennas']
            
            period_dict['seq_list'] = seq_list
            all_periods.append(period_dict)
            
        return all_periods
    
    def sequences_by_beam_antenna(self, beam_num=11, fsamp=None, lsamp=None,
                            offset=0,antenna=8, blank=False):
        
        # min_value = 1
        # ref_value = 2**15

        nant=self.data_records[beam_num]["nAntennas"]
        nsamp=self.data_records[beam_num]["nSamples"]

        if lsamp == None:
            lsamp = nsamp
        if fsamp == None:
            fsamp=0

        thresh=1.
        # pt_thresh=1.5


        seq_data_dict = {}
        seq_data_dict["fsamp"] = fsamp
        seq_data_dict["lsamp"] = lsamp
        seq_data_dict["nant"] = nant
        seq_data_dict["nsamp"] = nsamp

        avg_pwr=np.zeros(int(self.data_records[beam_num]["nSamples"]))
        int_seq=0
        avg_tot=0

        nnoise=int(0.1*nsamp)
        noise_pts=np.zeros(nnoise)
        noise_pts[:]=1.e8
        noise_tot=0
        for iSeq in range(int(self.data_records[beam_num]["nSeq"])):
            reals= np.real(self.data_records[beam_num]["seq_list"][iSeq]['samples'][antenna])
            imags= np.imag(self.data_records[beam_num]["seq_list"][iSeq]['samples'][antenna])
            pseq=reals**2 + imags**2
            for np0 in pseq:
                np_last=noise_pts[-1]
                for j in range(nnoise):
                    if np0 < noise_pts[j]:
                        break
                        
                hold=noise_pts[j]
                noise_pts[j]=np0
                for jj in range(j+1,nnoise):
                    hold0=noise_pts[jj]
                    noise_pts[jj]=hold
                    hold=hold0

                if noise_pts[-1] > np_last: noise_pts[-1]=np_last
            noise_tot+=np.sum(noise_pts)    
            noise_pts[:]=1.e8
            
        norm=noise_tot/(nsamp*self.data_records[beam_num]["nSeq"])


        sec0=self.data_records[beam_num]["seq_list"][0]["seq_start_time_sec"]
        msec0=self.data_records[beam_num]["seq_list"][0]["seq_start_time_usec"]
        t0=sec0+msec0/1.e6

        # dms=(self.data_records[beam_num]["seq_list"][1]["seq_start_time_usec"]-self.data_records[beam_num]["seq_list"][0]["seq_start_time_usec"])/1.e6

        seq_data_dict["nSeq"] = self.data_records[beam_num]["nSeq"]

        for iSeq in range(int(self.data_records[beam_num]["nSeq"])):

            sec=self.data_records[beam_num]["seq_list"][iSeq]["seq_start_time_sec"]
            msec=self.data_records[beam_num]["seq_list"][iSeq]["seq_start_time_usec"]
            tt=sec+msec/1.0e6
            # print(self.data_records[beam_num]["seq_list"][iSeq]["sequence_no_in_period"],tt-t0,(tt-t0)/dms)
            reals= np.real(self.data_records[beam_num]["seq_list"][iSeq]['samples'][antenna])
            imags= np.imag(self.data_records[beam_num]["seq_list"][iSeq]['samples'][antenna])
            pseq=reals**2 + imags**2

            pts=self.data_records[beam_num]["seq_list"][iSeq]['samples'][antenna]
            pseq_c=pts*np.conj(pts)
            
            noise_pts =  np.where((pseq<thresh))    
            #    if len(noise_pts[0]) > pt_thresh: 
            avg_pwr+= pseq
            int_seq+=1 
            samps=self.data_records[beam_num]["ppat"]*self.data_records[beam_num]["mpinc"]/self.data_records[beam_num]["smsep"]
                
            thresh=500.
            noise_pts =  np.where((avg_pwr<thresh))
            
            if blank:
                for j in range(len(samps)):
                    avg_pwr[int(samps[j])]=0
                    avg_pwr[int(samps[j]-2)]=0
                    avg_pwr[int(samps[j]-1)]=0
                    avg_pwr[int(samps[j]+1)]=0
                    avg_pwr[int(samps[j]+2)]=0
                    avg_pwr[int(samps[j]+3)]=0
                    avg_pwr[int(samps[j]+4)]=0
                    # avg_pwr[int(samps[j]+5)]=0
                    # avg_pwr[int(samps[j]+6)]=0
                
                    pseq[int(samps[j])]=0
                    pseq[int(samps[j]-1)]=0
                    pseq[int(samps[j]+1)]=0
                    pseq[int(samps[j]-2)]=0
                    pseq[int(samps[j]+2)]=0
                    pseq[int(samps[j]+3)]=0
                    pseq[int(samps[j]+4)]=0
                    pseq[int(samps[j]+5)]=0
                    pseq[int(samps[j]+6)]=0

            # ax.cla()

            seq_data_dict["seq_"+str(iSeq)] = {}
            seq_data_dict["seq_"+str(iSeq)]['reals'] = reals[fsamp:lsamp]+iSeq*offset
            seq_data_dict["seq_"+str(iSeq)]['imags'] = imags[fsamp:lsamp]+iSeq*offset
            
            seq_data_dict["seq_"+str(iSeq)]['pseq_c'] = np.absolute(pseq_c[fsamp:lsamp])
            
            # if plot_iq:
            #     ax.plot(reals[fsamp:lsamp]+iSeq*offset)
            #     ax.plot(imags[fsamp:lsamp]+iSeq*offset)
            # else:
            #     # ax.plot(pseq[fsamp:lsamp]+iSeq*offset,lw=0.5)
            #     ax.plot(np.absolute(pseq_c[fsamp:lsamp]))
            # if( mx_val!= None):ax.set_ylim(mn_val,mx_val) 

            # figname=fname+"_{:0d}.png".format(iSeq)
            # plt.savefig(figname,orientation='landscape',dpi=200)

            tot_pwr=np.sum(pseq)
            avg_tot += tot_pwr
            # print(tot_pwr,np.sum(pseq[fsamp:lsamp]/(lsamp-fsamp)))
            
                    
            # noise_sum=np.sum(avg_pwr[noise_pts])
            n_noise_pts=len(noise_pts[0])
            # print("Beam: ",beam_num," Antenna: ",antenna,"  NOISE POINTS: ",noise_sum/n_noise_pts,n_noise_pts)
            # print("NOISE Avg: ",noise_sum/n_noise_pts)


        avg_tot/=nsamp
        # nbins=100
        # binsz=2
        #avg_pwr/=avg_tot
        avg_pwr/=self.data_records[beam_num]["nSeq"]
        #avg_pwr/=norm
        # pwr_hist=np.zeros(nbins,int)
        # bins=np.arange(nbins)*binsz
        # for ib in range(nbins-1):
        #     for ir in range(len(avg_pwr)):
        #         if (avg_pwr[ir]>bins[ib]) and (avg_pwr[ir]<=bins[ib+1]):
        #             pwr_hist[ib]+=1
        seq_data_dict["avg_pwr"] = avg_pwr

        return seq_data_dict
    

    def summary_by_beam_antenna(self):

        nbeams=len(self.data_records)
        beam_num = 11 # Just selecting a randome beam here! Could be anything
        nant=self.data_records[beam_num]["nAntennas"]
        nsamp=self.data_records[beam_num]["nSamples"]
        summary_array=np.zeros((nbeams,nant))
        avg_pwr_values=np.zeros((nbeams,nant,nsamp))
        # print(nsamp,"....")

        thresh=100.
        pt_thresh=150

        summary_dict = {}

        summary_dict["nant"] = nant
        summary_dict["nsamp"] = nsamp
        summary_dict["nbeams"] = nbeams

        for jp in range(nbeams):
            for ja in range(nant):
                avg_pwr=np.zeros(int(self.data_records[jp]["nSamples"]))
                for iSeq in range(int(self.data_records[jp]["nSeq"])):
                    reals= np.real(self.data_records[jp]["seq_list"][iSeq]['samples'][ja])
                    imags= np.imag(self.data_records[jp]["seq_list"][iSeq]['samples'][ja])
                    pseq=reals**2 + imags**2
                    noise_pts =  np.where((avg_pwr<thresh))
                    if len(noise_pts[0]) > pt_thresh: 
                        avg_pwr+= pseq

                samps=self.data_records[jp]["ppat"]*self.data_records[jp]["mpinc"]/self.data_records[jp]["smsep"]

                thresh=500.
                noise_pts =  np.where((avg_pwr<thresh))

                for j in range(len(samps)):
                    avg_pwr[int(samps[j])]=0
                    avg_pwr[int(samps[j]-1)]=0
                    avg_pwr[int(samps[j]+1)]=0
                    avg_pwr[int(samps[j]-2)]=0
                    avg_pwr[int(samps[j]+2)]=0
                    avg_pwr[int(samps[j]+3)]=0
            
                # print(avg_pwr)
                # print(len(avg_pwr))
                avg_pwr_values[jp,ja,:] = avg_pwr


                noise_sum=np.sum(avg_pwr[noise_pts])
                n_noise_pts=len(noise_pts[0])
                # print("Beam: ",jp," Antenna: ",ja,"  NOISE POINTS: ",noise_sum,n_noise_pts)
                # print("NOISE Avg: ",noise_sum/n_noise_pts)
                summary_array[jp,ja]=noise_sum/n_noise_pts

        summary_dict["avg_pwr_vals"] = avg_pwr_values
        summary_dict["summary_array"] = summary_array

        return summary_dict
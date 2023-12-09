import numpy as np
# from sklearn.preprocessing import MinMaxScaler

class DF_Data():
    def __init__(self):
        self.timestamp = None
        self.f1 = 0
        self.f2 = 0
        self.n_samples = 0
        self.beam_width = 22.5
        self.alpha1 = -90
        self.alpha2 = 90
        self.n_sectors = int((self.alpha2 - self.alpha1)/self.beam_width)
        self.amplitudes = []
        self.angle_pt = 0
        self.heading = 0

    @property
    def f1(self):
        return self._f1
    
    @f1.setter
    def f1(self, value):
        self._f1 = value
    
    @property
    def f2(self):
        return self._f2
    
    @f2.setter
    def f2(self, value):
        self._f2 = value
    
    @property
    def n_samples(self):
        return self._n_samples
    
    @n_samples.setter
    def n_samples(self, value):
        self._n_samples = value

    @property
    def bandwidth(self):
        return self.f2 - self.f1
    
    @bandwidth.setter
    def bandwidth(self, value):
        self._bandwidth = value
    
    @property
    def beam_width(self):
        return self._beam_width
    
    @beam_width.setter
    def beam_width(self, value):
        self._beam_width = value
    
    @property
    def alpha1(self):
        return self._alpha1
    
    @alpha1.setter
    def alpha1(self, value):
        self._alpha1 = value
    
    @property
    def alpha2(self):
        return self._alpha2
    
    @alpha2.setter
    def alpha2(self, value):
        self._alpha2 = value
    
    @property
    def n_sectors(self):
        return int((self._alpha2 - self._alpha1)/self._beam_width)
    
    @n_sectors.setter
    def n_sectors(self, value):
        self._n_sectors = int(value)
    
    @property
    def amplitudes(self):
        return self._amplitudes

    @amplitudes.setter
    def amplitudes(self, values):
        self._amplitudes = values

    @property
    def angle_pt(self):
        return self._angle_pt
    
    @angle_pt.setter
    def angle_pt(self, value):
        self._angle_pt = value

    @property
    def heading(self):
        return self._heading
    
    @heading.setter
    def heading(self, value):
        self._heading = value

    @property
    def current_sector(self):
        return int((self._angle_pt - self._alpha1)/self._beam_width)

    @property
    def normalized_amplitudes(self):
        return [(1-(float(i)-min(self._amplitudes))/(max(self._amplitudes)-min(self._amplitudes))) for i in self._amplitudes]
    
    def initialize_matrix(self):
        self.matrix = np.zeros((self._n_samples, self.n_sectors+1))

    def normalize_matrix(self):
        # scaler = MinMaxScaler()
        # scaler.fit(self.matrix)
        # self.lp_matrix = 1 - scaler.transform(self.matrix)

        self.lp_matrix = (self.matrix - self.matrix.min()) / (self.matrix.max() - self.matrix.min())
        self.lp_matrix = 1 - self.lp_matrix
        self.lp_matrix = np.round(self.lp_matrix, 4)

    def radar_plot_data(self):
        rows, cols = self.lp_matrix.shape

        significant_frequencies = []
        significant_amplitudes = []
        significant_angles = []

        for c in range(cols):
            for r in range(rows):
                if self.lp_matrix[r][c] < 0.6:
                    significant_frequencies.append(self.f1 + (r * (self.bandwidth/self.n_samples)))
                    significant_amplitudes.append(self.lp_matrix[r][c] + 0.1)
                    significant_angles.append((c*self.beam_width)+self.alpha1)

        if len(significant_amplitudes) > 4:

            return (significant_frequencies[:4], significant_angles[:4], significant_amplitudes[:4])
        else:
            return (significant_frequencies, significant_angles, significant_amplitudes)

class Antenna_Dyanmaic():
    def __init__(self) -> None:
        self.amplitudes = []

    @property
    def amplitudes(self):
        return self._amplitudes
    
    @amplitudes.setter
    def amplitudes(self, values):
        self._amplitudes = values

class Antenna_Static():
    def __init__(self):
        
        self.f1 = 0
        self.f2 = 0
        self.n_samples = 0
        self.beam_width = 20
    
    @property
    def f1(self):
        return self._f1
    
    @property
    def f2(self):
        return self._f2
    
    @property
    def n_samples(self):
        return self._n_samples

    @property
    def bandwidth(self):
        return self.f2 - self.f1
    
    @property
    def beam_width(self):
        return self._beam_width
    
    @f1.setter
    def f1(self, value):
        self._f1 = value

    @f2.setter
    def f2(self, value):
        self._f2 = value

    @n_samples.setter
    def n_samples(self, value):
        self._n_samples = value

    @beam_width.setter
    def beam_width(self, value):
        self._beam_width = value

    @bandwidth.setter
    def bandwidth(self, value):
        self._bandwidth = value

if __name__ == '__main__':
    static_data = Antenna_Static()
    print(static_data.__dict__)

    dynamic_data = Antenna_Dyanmaic()
    print(dynamic_data.__dict__)
import numpy as np

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
        self.matrix = np.zeros((self._n_samples, self.n_sectors))

    def normalize_matrix(self):
        self.lp_matrix = (self.matrix - self.matrix.min()) / (self.matrix.max() - self.matrix.min())

if __name__ == '__main__':
    
    df_data = DF_Data()

    df_data.f1 = 400
    df_data.f2 = 590000
    df_data.n_samples = 10
    df_data.amplitudes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    frequencies = list(np.arange(start=df_data.f1, stop=df_data.f2, step=((df_data.f2-df_data.f1)/df_data.n_samples)))
    df_data.angle_pt = 0

    # print(df_data.current_sector)

    # print(np.zeros((df_data.n_samples, df_data.n_sectors)))

    df_data.initialize_matrix()

    df_data.matrix[:, df_data.current_sector] = df_data.amplitudes

    df_data.normalize_matrix()
    print(df_data.lp_matrix)
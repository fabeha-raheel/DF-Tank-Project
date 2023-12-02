class DF_Antenna_Data():
    def __init__(self):
        self.timestamp = None
        self.f1 = 0
        self.f2 = 0
        self.n_samples = 0
        self.amplitudes = []
        self.angle_pt = 0
        self.heading = 0
        self.beam_width = 0

    @property
    def bandwidth(self):
        return self.f2 - self.f1

    @property
    def n_samples(self):
        return self._n_samples

    @n_samples.setter
    def n_samples(self, value):
        self._n_samples = value

    @property
    def amplitudes(self):
        return self._amplitudes

    @amplitudes.setter
    def amplitudes(self, values):
        self._amplitudes = values

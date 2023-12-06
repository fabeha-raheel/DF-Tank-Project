class DF_Data_Dynamic():
    def __init__(self):
        self.timestamp = None
        self.amplitudes = []
        self.angle_pt = 0
        self.heading = 0

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

class DF_Data_Static():
    def __init__(self) -> None:
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
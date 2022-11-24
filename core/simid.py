

class SimID:
    
    def __init__(self, serid=-1, truid=-1, meaid=-1):
        self.set_new_id(serid, truid, meaid)


    def __eq__(self, other) -> bool:
        return all([
            self.tru == other.tru,
            self.mea == other.mea,
            self.ser == other.ser
            ]
        )
    

    def set_new_id(self, serid=-1, truid=-1, meaid=-1):
        self.ser = serid
        self.tru = truid
        self.mea = meaid
        

    def __repr__(self):
        return 'SimId : (ser={:> 5}, tru={:> 5}, mea={:> 5})'.format(self.ser, self.tru, self.mea)


    def get_ids(self):
        yield self.ser
        yield self.tru
        yield self.mea
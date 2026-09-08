uint64 timestamp        # time since system start (microseconds)

uint8 rotor_index       # which rotor was changed
float32 scale           # applied effectiveness scalar, 1.0 = nominal
float32[6] column       # that rotor's effectiveness column: Mx My Mz Fx Fy Fz

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/install/brain'

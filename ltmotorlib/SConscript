import rtconfig
from building import *

cwd = GetCurrentDir()
src = Glob('*.c')
CPPPATH = [cwd]
    
group = DefineGroup('ltmotorlib', src, depend = [''], CPPPATH = CPPPATH)

Return('group')

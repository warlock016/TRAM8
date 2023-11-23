print(' ')
print('**************************************')
print('ATMEL Hex to LPZW Sysex file converter')
print('V1.2')
print(' ')

import sys
import binascii
from intelhex import IntelHex

#get filename of the hex file
file_name = sys.argv[1]
raw_name = file_name.split('.')[0]
#print(file_name)
print('Convertion utility running for: ' + raw_name)
print(' ')

# total pages minus BL ##############################
# to avoid big sysex files!  

#for atmega64 i.e. Schleussig, EC Mute, RKoch w/large_boot_start: 
#lines_p_page = 16
#bootload_start = 0xE000

#for atmega8 i.e. WK4, TRAM8 w/ large_boot_start: 
lines_p_page = 4
bootload_start = 0xE000


# Naming and Version conventions for sysex data

#preamble for schleussig 
#sysex_preamble = 'LPZWSCHLFW'
#sysex_version  = binascii.unhexlify('0002')

#preamble for tram8
sysex_preamble = b'T8FW'
sysex_version  = b'\x00' 


#print('bootload_start: 0x{0:X}'.format(bootload_start))

code_end = bootload_start - 1 
#print(code_end)

print('Statistics:')
print('')

ih = IntelHex()                     # create empty object
ih.loadhex(file_name)               # load from hex
print('Last addr in hex file: 0x{0:X}'.format(len(ih)-1)) 


for i in range(code_end):
	if ih[i] != 255: #check when the code ends!  
		last_addr = i

print('Last used addr: 0x{0:X}'.format(last_addr))
print('Value at last used addr: 0x{0:X}'.format(ih[last_addr]))

num_pages = int(round(last_addr/(16*lines_p_page)+0.5)) #calculate number of pages to be written
print('Used flash pages in current code: {0:d}' .format(num_pages))

if last_addr < (bootload_start-1):
	print('free space in app section, adding one free page')
	num_pages = num_pages + 1
	for i in range(16*lines_p_page):
		last_addr = last_addr + 1
		ih[last_addr]= 0
		
	
line_file_length = num_pages*16*lines_p_page


linFileName = raw_name + '_linear.hex'
linFile = open(linFileName,'w')
ih[0:line_file_length].write_hex_file(linFile)
linFile.close()

print(' ')
print('Created hex file with linear address space: ' + linFileName)
print('Last written addr: 0x{0:X}'.format(line_file_length))

sysex_start = bytes.fromhex('F000297F')
sysex_endhex = bytes.fromhex('3A303030303030303146460D0AF7')
sysex_fill = (b'\x00' * 451) #ascii zeros for processor downtime while flashing

hexFile = open(linFileName)
sysexFileName = raw_name + '_firmware.syx'
sysexFile = open(sysexFileName,'wb')

sysexFile.write(sysex_start)
sysexFile.write(sysex_preamble)
sysexFile.write(sysex_version)


for i in range(num_pages):
	sysexFile.write(sysex_fill)
	for j in range(lines_p_page):
			line = hexFile.readline()
			sysexFile.write(line.encode("utf8"))

sysexFile.write(sysex_fill)
sysexFile.write(sysex_endhex)

sysexFile.close()

print(' ')
print('Created Sysex file for MIDI upgrade: ' + sysexFileName)
print(' ')
print('DONE!')
print(' ')

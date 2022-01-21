 # The magic header half-word at the start of each packet.
MAGIC_WORD = 0x057B
MSG_MAGIC_WORD = 0x057C

# The struct format of the header.
HEADER_FORMAT = '<HBBL'

# The struct format of the header.
#https://docs.python.org/3/library/array.html
MSG_HEADER_FORMAT = '<HBHL'

# All supported formats by this reception script. The items in this dict 
# are functions that return the struct deserialization code to unpack 
# a single batch and the number of different data fields in a batch.
STREAM_FORMAT = {
    # dummy entry, information is gathered from application settings
    0: lambda batch_size: \
        (f'<{batch_size}H{batch_size}H{batch_size}H{batch_size}H', 4), 
    # uint16; uint16; uint16; uint16 
    1: lambda batch_size: \
        (f'<{batch_size}H{batch_size}H{batch_size}H{batch_size}H', 4),
    # int16; int16; uint16; uint16   
    2: lambda batch_size: \
        (f'<{batch_size}h{batch_size}h{batch_size}H{batch_size}H', 4),
    
}

# dictionary of supported gains of Stabilizer's AFEs 
GAINS = {
    'G1': 1,
    'G2': 2,
    'G5': 5,
    'G10': 10
}
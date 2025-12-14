class _CtrlTableValue():

    def __init__(self, address, length_bytes, access):

        def validate_address(address):
            if not isinstance(address, int):
                raise ValueError(f'The given address {address} is not of type int')

        def valididate_len_bytes(length_bytes):
            if not isinstance(length_bytes, int):
                raise ValueError(f'The given length_bytes {length_bytes} is not of type int')
            valid_lengths_bytes = {1,2,4}
            if not(length_bytes in valid_lengths_bytes):
                raise ValueError(f'The given length_bytes {length_bytes} is not valid. Must be in {valid_lengths_bytes}')
            
        def validate_access(access):
            valid_access_strings= {'RW',"W","R"}
            if not (access in valid_access_strings):
                raise ValueError(f'The given access {access} is not valid. Must be in {valid_access_strings}')

        validate_address(address)
        valididate_len_bytes(length_bytes)
        validate_access(access)

        self.address = address 
        self.length_bytes = length_bytes
        self.access = access 
from hashlib import sha256
from Crypto import Random
from Crypto.PublicKey import RSA

PATH_TO_KEYS = './keys'
KEY_SIZE = 1024

def parse_mission(mission: str) -> list:
    cmds = mission.split('&')
    for idx, cmd in enumerate(cmds):
        cmds[idx] = [cmd[0], *cmd[1:].split('_')]
    return cmds

def sign(message: str, key_group: str) -> int:
    n, d = get_key(key_group, private=True)
    msg_bytes = message.encode()
    hash = int.from_bytes(sha256(msg_bytes).digest(), byteorder='big', signed=False)
    signature = pow(hash, d, n)
    
    return signature

def verify(message: str, signature: int, key_group: str) -> bool:
    n, e = get_key(key_group, private=False)
    msg_bytes = message.encode()
    hash = int.from_bytes(sha256(msg_bytes).digest(), byteorder='big', signed=False)
    hashFromSignature = pow(signature, e, n)
    return hash == hashFromSignature

def get_key(key_group: str, private: bool) -> list:
    path = f'{PATH_TO_KEYS}/{key_group}.txt'
    with open(path, 'r') as kf:
        data = kf.read()
        keys = data.split('\n')
        keys = list(map(lambda k: int(k, 16), keys))
    if len(keys) == 3 and private:
        return keys[0], keys[2]
    else:
        return keys[:2]

def generate_keys(keysize: int) -> list:
    random_generator = Random.new().read
    key = RSA.generate(keysize, random_generator)
    private, public = key, key.publickey()
    return public, private

def save_keys(public: RSA.RsaKey, private: RSA.RsaKey, key_group: str) -> None:
    path = f'{PATH_TO_KEYS}/{key_group}.txt'
    lines_to_write = list(map(hex, [public.n, public.e, private.d]))
    with open(path, 'w') as kf:
        kf.write("\n".join(lines_to_write))

def save_public_key(n: str, e: str, key_group: str) -> None:
    path = f'{PATH_TO_KEYS}/{key_group}.txt'
    lines_to_write = [n, e]
    with open(path, 'w') as kf:
        kf.write("\n".join(lines_to_write))
def bytes2array(bts):
    result=[]
    for i in bts:
        result.append(i)
    return result

def read_key(file_name):

    headerfile="""
    #ifndef __DRONE_KEY_H
    #define __DRONE_KEY_H

    char tx_secretkey[]={%s};
    char rx_pubkey[] = { %s }; 
    #endif
    """
    with open(file_name,"rb") as f:
        drone_secret=f.read(32)
        gs_pub=f.read(32)
        sec_s=",".join( str(x) for x in drone_secret)
        pub_s=",".join( str(x) for x in gs_pub)
     
    a=headerfile %(sec_s,pub_s)
    with open("main/key.h","w+") as f:
        f.write(a)
    print(a)

read_key("drone.key")
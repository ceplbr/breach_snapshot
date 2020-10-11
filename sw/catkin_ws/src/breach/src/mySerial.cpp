#include "mySerial.h"

unsigned char char2byte(char c) {
    unsigned char cr = ( c < 0 ) ? c + 256 : c;
    return cr;
}

Message::Message()
{
}

Message::Message(int o)
{
    id = 0;
    mess_len = 0;
    data_type = 0;
    data_char = 0;
    data_int = 0;
    data_uint = 0;
    data_long = 0;
    data_ulong = 0;
    data_float = 0.0;
}

//
// Destructor
//
Message::~Message()
{
}

MySerial::MySerial(std::string port, int baud, int timeout)
    :dev(port, baud, serial::Timeout::simpleTimeout(timeout))
{

}

int MySerial::receive(uint8_t *cc) {

    /*
    uint8_t start, len, data_type, checksum;
    uint32_t check += 0xFO;

    // read len
    len = mbed.read();
    // check len
    if (!mbed.isValid() || len > PC_LEN_MAX) {
        return false;
    }
    check += len;

    // read data_type
    data_type = mbed.read();
    // check
    if (!mbed.isValid() || len > PC_LEN_MAX) {
        return false;
    }
    check += data_type;

    // create data buffer
    char c[PC_LEN_MAX];
    // read data
    for (int i = 0; i < len; i++) {
        c[i] = mbed.read();
        if (!mbed.isValid()) {
            return false;
        }
        check += c[i];
    }

    // read checksum
    data_type = mbed.read();
    // check
    if (!mbed.isValid() || len > PC_LEN_MAX) {
        return false;
    }
    if ( (uint8_t)(check%256) != checksum ) {
        return false;
    }


    // data_type
    data_type = (uint8_t)c[0];

    switch (data_type) {
        case TYPE_CHAR:
            break;
        case TYPE_UINT16:
            break;
        case TYPE_INT16:
            break;
        case TYPE_UINT32:
            break;
        case TYPE_INT32:
            break;
        case TYPE_UINT64:
            break;
        case TYPE_FLOAT32:
            break;
        case TYPE_STRING:
            break;
    }


    return true;
    */


    /*
    unsigned char c[12];

    for( int i = 0; i < 12; i++)
        c[i] = char2byte(cc[i]);

    checksum = 0;
    for (int i = 0; i < 11; i++)
        checksum += (int)c[i];
    checksum = checksum % 256;
    if (checksum == c[11]) {
        mess.mess_len        = (c[0] > 8) ? 8 : c[0];
        mess.data_type  = c[1];
        mess.id         = c[2];
        if ( mess.data_type == 1 )
            mess.data_int = (int)c[3];
        else if (mess.data_type == 2) {
            mess.data_int=(int)c[3]<<24;
            mess.data_int|=(int)c[4]<<16;
            mess.data_int|=(int)c[5]<<8;
            mess.data_int|=(int)c[6];
        }
        else if (mess.data_type == 3) {
            char data[mess.mess_len];
            for (int j = 0; j < mess.mess_len; j++) {
                data[j] = c[3+j];
            }
            mess.data_float = atof(data);
        }
        return 1;
    }
    */
    return 0;
}

bool MySerial::send(uint8_t value, uint8_t id) {
    uint8_t buf[6];

    buf[0] = 0xF0;
    buf[1] = SIZE_INT8;
    buf[2] = TYPE_UINT8;
    buf[3] = id;
    buf[4] = value;
    buf[5] = SIZE_INT8+TYPE_UINT8+id+value;

    dev.write(buf, 6);

    return true;
}

bool MySerial::send(int8_t value, uint8_t id) {
    uint8_t buf[6];

    buf[0] = 0xF0;
    buf[1] = SIZE_INT8;
    buf[2] = TYPE_INT8;
    buf[3] = id;
    buf[4] = value;
    buf[5] = SIZE_INT8+TYPE_INT8+id+value;

    dev.write(buf, 6);

    return true;
}

bool MySerial::send(uint16_t value, uint8_t id) {
    uint8_t buf[7];
    uint8_t buf_value[2];
    pack_uint16(value, buf_value);

    buf[0] = 0xF0;
    buf[1] = SIZE_INT16;
    buf[2] = TYPE_UINT16;
    buf[3] = id;
    buf[4] = buf_value[0];
    buf[5] = buf_value[1];
    buf[6] = SIZE_INT16+TYPE_UINT16+id+buf_value[0]+buf_value[1];

    dev.write(buf, 7);

    return true;
}

bool MySerial::send(int16_t value, uint8_t id) {
    uint8_t buf[7];
    uint8_t buf_value[2];
    pack_sint16(value, buf_value);

    buf[0] = 0xF0;
    buf[1] = SIZE_INT16;
    buf[2] = TYPE_INT16;
    buf[3] = id;
    buf[4] = buf_value[0];
    buf[5] = buf_value[1];
    buf[6] = SIZE_INT16+TYPE_INT16+id+buf_value[0]+buf_value[1];

    dev.write(buf, 7);

    return true;
}

bool MySerial::send(uint32_t value, uint8_t id) {
    uint8_t buf[9];
    uint8_t buf_value[4];
    pack_uint32(value, buf_value);

    buf[0] = 0xF0;
    buf[1] = SIZE_INT32;
    buf[2] = TYPE_UINT32;
    buf[3] = id;
    buf[4] = buf_value[0];
    buf[5] = buf_value[1];
    buf[6] = buf_value[2];
    buf[7] = buf_value[3];
    buf[8] = buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7];

    dev.write(buf, 9);

    return true;
}

bool MySerial::send(int32_t value, uint8_t id) {
    uint8_t buf[9];
    uint8_t buf_value[4];
    pack_sint32(value, buf_value);

    buf[0] = 0xF0;
    buf[1] = SIZE_INT32;
    buf[2] = TYPE_INT32;
    buf[3] = id;
    buf[4] = buf_value[0];
    buf[5] = buf_value[1];
    buf[6] = buf_value[2];
    buf[7] = buf_value[3];
    buf[8] = SIZE_INT32+TYPE_INT32+id+buf_value[0]+buf_value[1]+buf_value[2]+buf_value[3];

    dev.write(buf, 9);

    return true;
}

bool MySerial::send(uint64_t value, uint8_t id) {
    uint8_t buf[13];
    uint8_t buf_value[8];
    pack_uint64(value, buf_value);

    buf[0] = 0xF0;
    buf[1] = SIZE_INT64;
    buf[2] = TYPE_UINT64;
    buf[3] = id;
    buf[4] = buf_value[0];
    buf[5] = buf_value[1];
    buf[6] = buf_value[2];
    buf[7] = buf_value[3];
    buf[8] = buf_value[4];
    buf[9] = buf_value[5];
    buf[10] = buf_value[6];
    buf[11] = buf_value[7];
    buf[12] = SIZE_INT64+TYPE_UINT64+id+buf_value[0]+buf_value[1]+buf_value[2]+buf_value[3]+buf_value[4]+buf_value[5]+buf_value[6]+buf_value[7];

    dev.write(buf, 13);

    return true;
}

bool MySerial::send(int64_t value, uint8_t id) {
    uint8_t buf[13];
    uint8_t buf_value[8];
    pack_sint64(value, buf_value);

    buf[0] = 0xF0;
    buf[1] = SIZE_INT64;
    buf[2] = TYPE_INT64;
    buf[3] = id;
    buf[4] = buf_value[0];
    buf[5] = buf_value[1];
    buf[6] = buf_value[2];
    buf[7] = buf_value[3];
    buf[8] = buf_value[4];
    buf[9] = buf_value[5];
    buf[10] = buf_value[6];
    buf[11] = buf_value[7];
    buf[12] = SIZE_INT64+TYPE_INT64+id+buf_value[0]+buf_value[1]+buf_value[2]+buf_value[3]+buf_value[4]+buf_value[5]+buf_value[6]+buf_value[7];

    dev.write(buf, 13);

    return true;
}


void MySerial::send(breach::serial_msg mess) {
    int err;
    unsigned char data[13];

    int check = mess.data_type + mess.id;
    int size = 0;
    char buff[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    switch (mess.data_type) {
        case 1:
            break;
        case 2:
            for(int i = 0; i < 4; i++)
                buff[i] = (uint8_t)((mess.data_int & (0xff000000 >> 8*i)) >> 8*(3-i));
            size = 4;
            break;
        case 3:
            size = sprintf(buff, "%f", mess.data_float);
            break;
    }
    for (int i = 0; i < 8; i++)
        check += buff[i];
    if(size > 8)
        size = 8;
    check += size;

    //printf("%d %d \n", (char)0xf0, (char)size);


    data[0] = (unsigned char)0xf0;
    data[1] = (unsigned char)size;
    data[2] = (unsigned char)mess.data_type;
    data[3] = (unsigned char)mess.id;
    for (int i = 0; i < 8; i++)
        data[4 + i] = (unsigned char)buff[i];
    data[12] = (unsigned char)(check%256);

    dev.write(data, 13);

    /*
    for( int i = 0; i < 13; i++)
        printf("%d ", data[i]);
    printf("\n");
    */
}


Message MySerial::getData() {
    return mess;
}



// +---------------------------------+
// |  PACKING                        |
// +---------------------------------+

// === PACK =================
bool MySerial::pack_uint8(uint8_t value, uint8_t *b)
{
    b[0] = value;

    return true;
}


bool MySerial::pack_sint8(int8_t value, uint8_t *b)
{
    b[0] = value;

    return true;
}

bool MySerial::pack_uint16(uint16_t value, uint8_t *b)
{
    uint16_t mask = 0xff;

    b[0] = (value >> 8) & mask;
    b[1] = value & mask;

    return true;
}


bool MySerial::pack_sint16(int16_t value, uint8_t *b)
{
    int16_t mask = 0xff;

    b[0] = (value >> 8) & mask;
    b[1] = value & mask;

    return true;
}


bool MySerial::pack_uint32(uint32_t value, uint8_t *b)
{
    uint32_t mask = 0xff;

    b[0] = (value >> 24) & mask;
    b[1] = (value >> 16) & mask;
    b[2] = (value >> 8) & mask;
    b[3] = value & mask;

    return true;
}

bool MySerial::pack_sint32(int32_t value, uint8_t *b)
{
    int32_t mask = 0xff;

    b[0] = (value >> 24) & mask;
    b[1] = (value >> 16) & mask;
    b[2] = (value >> 8) & mask;
    b[3] = value & mask;

    return true;
}

bool MySerial::pack_uint64(uint64_t value, uint8_t *b)
{
    uint32_t mask = 0xff;

    b[0] = (value >> 56) & mask;
    b[1] = (value >> 48) & mask;
    b[2] = (value >> 40) & mask;
    b[3] = (value >> 32) & mask;
    b[4] = (value >> 24) & mask;
    b[5] = (value >> 16) & mask;
    b[6] = (value >> 8) & mask;
    b[7] = value & mask;

    return true;
}

bool MySerial::pack_sint64(int64_t value, uint8_t *b)
{
    int32_t mask = 0xff;

    b[0] = (value >> 56) & mask;
    b[1] = (value >> 48) & mask;
    b[2] = (value >> 40) & mask;
    b[3] = (value >> 32) & mask;
    b[4] = (value >> 24) & mask;
    b[5] = (value >> 16) & mask;
    b[6] = (value >> 8) & mask;
    b[7] = value & mask;

    return true;
}

bool MySerial::pack_float32(float value, uint8_t *b)
{
    double  temp_float = value * FIX16_ONE;
    temp_float += (temp_float >= 0) ? 0.5 : -0.5;
    int32_t temp_int   = int32_t(temp_float);
    return pack_sint32(temp_int, b);
}

// === UNPACK =========================

uint8_t MySerial::unpack_uint8(uint8_t *b)
{
    uint8_t     value;

    value = (uint8_t)b[0];

    return value;
}

int8_t MySerial::unpack_sint8(uint8_t *b)
{
    int8_t     value;

    value = (int8_t)b[0];

    return value;
}

uint16_t MySerial::unpack_uint16(uint8_t *b)
{
    uint16_t     value;

    value = (uint16_t)b[0]<<(8);
    value |= (uint16_t)b[1];

    return value;
}

int16_t MySerial::unpack_sint16(uint8_t *b)
{
    int16_t     value;

    value = (int16_t)b[0]<<(8);
    value |= (int16_t)b[1];

    return value;
}

uint32_t MySerial::unpack_uint32(uint8_t *b)
{
    uint32_t value = 0;

    int j = 0;
    for (int i = 3; i >= 0; i--) {
        value |= (uint32_t)b[j]<<(8*i);
        j++;
    }

    return value;
}

int32_t MySerial::unpack_sint32(uint8_t *b)
{
    int32_t value = 0;

    value = ((int32_t)b[0]) << 24;
    value |= ((int32_t)b[1]) << 16;
    value |= ((int32_t)b[2]) << 8;
    value |= ((int32_t)b[3]);

#if 0
    int j = 0;
    for (int i = 3; i >= 0; i--) {
        value |= (int32_t)b[j]<<(8*i);
        j++;
    }
#endif

    return value;
}

uint64_t MySerial::unpack_uint64(uint8_t *b)
{
    uint64_t value = 0;

    int j = 0;
    for (int i = 7; i >= 0; i--) {
        value |= (uint64_t)b[j]<<(8*i);
        j++;
    }

    return value;
}

int64_t MySerial::unpack_sint64(uint8_t *b)
{
    int64_t value = 0;

    int j = 0;
    for (int i = 7; i >= 0; i--) {
        value |= (int64_t)b[j]<<(8*i);
        j++;
    }

    return value;
}

float MySerial::unpack_float32(uint8_t *b)
{
    int32_t temp = unpack_sint32(b);
    return float(temp) / FIX16_ONE;
}


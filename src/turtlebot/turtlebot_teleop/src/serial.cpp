
//#include <sstream>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "serial.h"


// シリアルポートが開いているか判定
bool Serial::IsOpen(){
  if (m_fd < 0) return false;
  return true;
}
  // シリアルポートの初期化
bool Serial::Open( char *device_name, int bau ){
  /*
   読み書きのためにモデムデバイスをオープンする．ノイズによって CTRL-C
   がたまたま発生しても接続が切れないように，tty 制御はしない．
   */
  m_fd = open( device_name, O_RDWR | O_NOCTTY );  // デバイスオープン

  if (m_fd < 0) {
    perror(device_name);
    return false;
  }

  struct termios tio; // シリアル通信の値をセット出来る構造体
  memset(&tio, 0, sizeof(tio)); // メモリ確保

  tio.c_cflag = CS8 | CLOCAL | CREAD;
  /*
   BAUDRATE: ボーレートの設定．cfsetispeed と cfsetospeed も使用できる．
   CS8     : 8n1 (8 ビット，ノンパリティ，ストップビット 1)
   CLOCAL  : ローカル接続，モデム制御なし
   CREAD   : 受信文字(receiving characters)を有効にする．
   */

//  tio.c_cc[VTIME] = 200;  /* キャラクタ間タイマは未使用 */
  tio.c_cc[VMIN] = 15;    /* 何文字受け取ったらreadが返るか */

  /*
   ICANON  : カノニカル入力(行単位入力）
   */
  tio.c_lflag = ~ICANON;  // 無効
  //tio.c_lflag = ICANON; // 有効

  /*
   IGNPAR  : パリティエラーのデータは無視する
   ICRNL   : CR を NL に対応させる(これを行わないと，他のコンピュータで
   CR を入力しても，入力が終りにならない)
   それ以外の設定では，デバイスは raw モードである(他の入力処理は行わない)
   */
  tio.c_iflag = IGNPAR | ICRNL;

  // ボーレートの設定
  if( bau < 0 || bau > 115200 ){
    printf("invalied baudlate");
    return false;
  }
  
  m_bau = bau;
  cfsetispeed( &tio, m_bau );  // インプット
  cfsetospeed( &tio, m_bau );  // アウトプット
  // デバイスに設定を行う
  tcsetattr( m_fd, TCSANOW, &tio );
  
  return true;
}

bool Serial::Recieve( char *rec_buf, int len ){
  int rcv_cnt = -1;

  rcv_cnt = read( m_fd, rec_buf, len );

  if ( rcv_cnt < 0 ) {
    fprintf(stdout, "CHILD:Could not read from serial port\n");
    return false;
  }
  return true;
}

bool Serial::Send( char *send_buf, int len ){
  int write_cnt = -1;

  write_cnt = write( m_fd, send_buf, len );

  if (write_cnt < 0) {
    fprintf(stdout, "Could not write to serial port %d\n", write_cnt);
    return false;
  }
  return true;
}


bool Serial::Close(){
  fprintf(stdout, "Close port \n");
  close(m_fd);
  return true;
}


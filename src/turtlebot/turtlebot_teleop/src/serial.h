//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <string.h>
//#include <termios.h>
//#include <time.h>
//#include <sys/wait.h>

#ifndef TECH_SERIAL
#define TECH_SERIAL

class Serial{
public:
  Serial(){
    m_fd = -1;
  }
  ~Serial(){
    //Close();
  }

  bool Open( char *device_name, int bau );  // 引数は多分ストリームへのポインタ
  bool Recieve( char *rec_buf, int len );
  bool Send( char *send_buf, int len );
  bool Close();
  bool IsOpen();

private:
  int m_fd;   // ファイルポインタ
  int m_bau;  // ボーレート
};

#endif


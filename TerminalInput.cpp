#include "TerminalInput.h"

#include <cerrno>
#include <stdexcept>

#include <poll.h>
#include <unistd.h>

TerminalInput::TerminalInput()
{
    // STDIN_FILENO : 표준 입력(stdin)의 파일 디스크립터. 보통 값은 0
    // isatty(STDIN_FILENO) : 현재 표준 입력이 진짜 터미널인지 확인
    if (!isatty(STDIN_FILENO))
        throw std::runtime_error("Keyboard input requires terminal");

    // tcgetattr() : 현재 터미널 설정을 읽어오는 함수
    // 현재 터미널 설정을 original_에 저장
    if (tcgetattr(STDIN_FILENO, &original_) < 0)
        throw std::runtime_error("tcgetattr failed");

    // termios : Linux/Unix에서 터미널 설정을 담는 구조체
    // 원본 설정을 raw에 복사함
    termios raw = original_;

    // ICANON(canonical mode) ON : Enter를 눌러야 입력 전달
    // ECHO : 입력한 글자를 화면에 다시 보여주는 기능
    // 기존 설정은 유지하면서 ICANON과 ECHO 플래그만 끈다.
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));

    // VMIN : read()가 반환하기 전에 최소 몇 바이트의 입력을 기다릴 것인가
    // 입력이 하나도 없어도 read()가 계속 기다리지 않도록 함
    raw.c_cc[VMIN] = 0;

    // VTIME : 터미널의 read()에 적용되는 대기 시간 관련 설정
    // read() 자체에서 추가적인 시간 대기를 하지 않도록 설정
    raw.c_cc[VTIME] = 0;

    // tcsetattr() : 변경한 터미널 설정을 실제 터미널에 적용하는 함수
    // TCSANOW : 설정을 즉시 적용
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0)
        throw std::runtime_error("tcsetattr failed");

    // 터미널 설정이 정상적으로 변경되었음을 표시
    active_ = true;
}

TerminalInput::~TerminalInput()
{
    // 터미널 설정을 변경한 상태라면
    if (active_)
    {
        // 생성자에서 original_에 저장해둔 원래 터미널 설정으로 복구
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
}

int TerminalInput::readKey(int timeoutMs)
{
    // poll()에서 감시할 파일 디스크립터와 이벤트 정보를 저장하는 구조체
    pollfd descriptor{};

    // 표준 입력(stdin)을 감시하도록 설정
    descriptor.fd = STDIN_FILENO;

    // POLLIN : 읽을 수 있는 데이터가 들어오는 이벤트를 감시
    descriptor.events = POLLIN;

    // 표준 입력에 데이터가 들어오는지 timeoutMs 동안 대기
    // 두 번째 인자 1 : 감시할 pollfd 구조체가 1개라는 의미
    int result = poll(&descriptor, 1, timeoutMs);

    // poll() 실행 중 오류가 발생한 경우
    if (result < 0)
    {
        // EINTR : 시그널에 의해 poll()이 중간에 중단된 경우
        // 실제 키 입력이 없었던 것으로 처리
        if (errno == EINTR)
            return -1;

        // 그 외의 poll 오류는 예외 발생
        throw std::runtime_error("poll failed");
    }

    // result == 0 : timeoutMs 동안 입력이 들어오지 않은 경우
    // revents : 실제로 발생한 이벤트
    // POLLIN 이벤트가 발생하지 않았다면 읽을 키가 없는 것으로 처리
    if (result == 0 || !(descriptor.revents & POLLIN))
        return -1;

    // 입력받은 키 1바이트를 저장할 변수
    unsigned char key = 0;

    // 표준 입력에서 1바이트를 읽어 key에 저장
    // 정상적으로 1바이트를 읽었다면 key를 int형으로 변환하여 반환
    if (read(STDIN_FILENO, &key, 1) == 1)
        return static_cast<int>(key);

    // 키를 정상적으로 읽지 못했다면 -1 반환
    return -1;
}
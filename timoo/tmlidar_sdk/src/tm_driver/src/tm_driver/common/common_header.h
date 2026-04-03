#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
/*Common*/
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <memory>
#include <array>
#include <algorithm>
#include <functional>
#include <iterator>
#include <chrono>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <stdexcept>
#include <mutex>
#include <type_traits>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <tm_driver/macro/version.h>
/*Linux*/
#ifdef __linux__
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#elif _WIN32
#include <winsock2.h>
#include <windows.h>
#endif

/*Eigen*/
#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#if defined(_WIN32)
#include <io.h>
#include <windows.h>
inline void setConsoleColor(WORD c)
{
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  SetConsoleTextAttribute(hConsole, c);
}
#endif

/*Pcap*/
#include <pcap.h>

/*Camera*/
typedef std::pair<std::string, double> CameraTrigger;

/*Packet Length*/
const size_t MECH_PKT_LEN = 1206;

/*Output style*/
#ifndef TM_INFOL
#if defined(_WIN32)
inline std::ostream& _TM_INFOL()
{
  setConsoleColor(FOREGROUND_GREEN);
  return std::cout;
}
#define TM_INFOL _TM_INFOL()
#else
#define TM_INFOL (std::cout << "\033[32m")
#endif
#endif

#ifndef TM_INFO
#if defined(_WIN32)
inline std::ostream& _TM_INFO()
{
  setConsoleColor(FOREGROUND_GREEN | FOREGROUND_INTENSITY);
  return std::cout;
}
#define TM_INFO _TM_INFO()
#else
#define TM_INFO (std::cout << "\033[1m\033[32m")
#endif
#endif

#ifndef TM_WARNING
#if defined(_WIN32)
inline std::ostream& _TM_WARNING()
{
  setConsoleColor(FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_INTENSITY);
  return std::cout;
}
#define TM_WARNING _TM_WARNING()
#else
#define TM_WARNING (std::cout << "\033[1m\033[33m")
#endif
#endif

#ifndef TM_ERROR
#if defined(_WIN32)
inline std::ostream& _TM_ERROR()
{
  setConsoleColor(FOREGROUND_RED | FOREGROUND_INTENSITY);
  return std::cout;
}
#define TM_ERROR _TM_ERROR()
#else
#define TM_ERROR (std::cout << "\033[1m\033[31m")
#endif
#endif

#ifndef TM_DEBUG
#if defined(_WIN32)
inline std::ostream& _TM_DEBUG()
{
  setConsoleColor(FOREGROUND_GREEN);
  return std::cout;
}
#define TM_DEBUG _TM_DEBUG()
#else
#define TM_DEBUG (std::cout << "\033[1m\033[36m")
#endif
#endif

#ifndef TM_TITLE
#if defined(_WIN32)
inline std::ostream& _TM_TITLE()
{
  setConsoleColor(FOREGROUND_BLUE | FOREGROUND_RED | FOREGROUND_INTENSITY);
  return std::cout;
}
#define TM_TITLE _TM_TITLE()
#else
#define TM_TITLE (std::cout << "\033[1m\033[35m")
#endif
#endif

#ifndef TM_MSG
#if defined(_WIN32)
inline std::ostream& _TM_MSG()
{
  setConsoleColor(FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
  return std::cout;
}
#define TM_MSG _TM_MSG()
#else
#define TM_MSG (std::cout << "\033[1m\033[37m")
#endif
#endif

#ifndef TM_REND
#if defined(_WIN32)
inline std::ostream& TM_REND(std::ostream& stream)
{
  stream << std::endl;
  setConsoleColor(-1);
  return stream;
}
#else
#define TM_REND "\033[0m" << std::endl
#endif
#endif

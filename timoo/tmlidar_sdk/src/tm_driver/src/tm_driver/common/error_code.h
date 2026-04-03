#pragma once
#include <tm_driver/common/common_header.h>
namespace timoo
{
namespace lidar
{
/**
 * @brief Error Code for Timoo LiDAR Driver.
 * 0x00 For success info
 * 0x01 ~ 0x40 for Infos, some infomation during the program running
 * 0x41 ~ 0x80 for Warning, the program may not work normally
 * 0x81 ~ 0xC0 for Critical Error, the program will exit
 */
enum class ErrCodeType
{
  INFO_CODE,     ///< Common information
  WARNING_CODE,  ///< Program may not work normally
  ERROR_CODE     ///< Program will exit immediately
};
enum ErrCode
{
  ERRCODE_SUCCESS = 0x00,          ///< Normal
  ERRCODE_PCAPREPEAT = 0x01,       ///< Pcap file will play repeatedly
  ERRCODE_PCAPEXIT = 0x02,         ///< Pcap thread will exit
  ERRCODE_MSOPTIMEOUT = 0x41,      ///< Msop packets receive overtime (1 sec)
  ERRCODE_DIFOPTIMEOUT = 0x42,     ///< Difop packets receive overtime (2 sec)
  ERRCODE_MSOPINCOMPLETE = 0x43,   ///< Incomplete msop packets received
  ERRCODE_DIFOPINCOMPLETE = 0x44,  ///< Incomplete difop packets received
  ERRCODE_NODIFOPRECV = 0x45,      ///< Point cloud decoding process will not start until the difop packet receive
  ERRCODE_ZEROPOINTS = 0x46,       ///< Size of the point cloud is zero
  ERRCODE_ZEROIMU = 0x47,       ///< Size of the point cloud is zero
  ERRCODE_STARTBEFOREINIT = 0x48,  ///< start() function is called before initializing successfully
  ERRCODE_PCAPWRONGPATH = 0x49,    ///< Input directory of pcap file is wrong
  ERRCODE_MSOPPORTBUZY = 0x50,     ///< Input msop port is already used
  ERRCODE_DIFOPPORTBUZY = 0x51,    ///< Input difop port is already used
  ERRCODE_WRONGPKTHEADER = 0x52,   ///< Packet header is wrong
  ERRCODE_PKTNULL = 0x53,          ///< Input packet is null
  ERRCODE_PKTBUFOVERFLOW = 0x54    ///< Packet buffer is over flow
};

struct Error
{
  ErrCode error_code;
  ErrCodeType error_code_type;
  explicit Error(const ErrCode& code) : error_code(code)
  {
    if (error_code <= 0x40)
    {
      error_code_type = ErrCodeType::INFO_CODE;
    }
    else if (error_code <= 0x80)
    {
      error_code_type = ErrCodeType::WARNING_CODE;
    }
    else
    {
      error_code_type = ErrCodeType::ERROR_CODE;
    }
  }
  std::string toString() const
  {
    switch (error_code)
    {
      case ERRCODE_PCAPWRONGPATH:
        return "ERRCODE_PCAPWRONGPATH";
      case ERRCODE_MSOPPORTBUZY:
        return "ERRCODE_MSOPPORTBUZY";
      case ERRCODE_DIFOPPORTBUZY:
        return "ERRCODE_DIFOPPORTBUZY";
      case ERRCODE_PCAPREPEAT:
        return "Info_PcapRepeat";
      case ERRCODE_PCAPEXIT:
        return "Info_PcapExit";
      case ERRCODE_MSOPTIMEOUT:
        return "ERRCODE_MSOPTIMEOUT";
      case ERRCODE_DIFOPTIMEOUT:
        return "ERRCODE_DIFOPTIMEOUT";
      case ERRCODE_MSOPINCOMPLETE:
        return "ERRCODE_MSOPINCOMPLETE";
      case ERRCODE_DIFOPINCOMPLETE:
        return "ERRCODE_DIFOPINCOMPLETE";
      case ERRCODE_NODIFOPRECV:
        return "ERRCODE_NODIFOPRECV";
      case ERRCODE_ZEROPOINTS:
        return "ERRCODE_ZEROPOINTS";
       case ERRCODE_ZEROIMU:
        return "ERRCODE_ZEROIMU";
      case ERRCODE_STARTBEFOREINIT:
        return "ERRCODE_STARTBEFOREINIT";
      case ERRCODE_WRONGPKTHEADER:
        return "ERRCODE_WRONGPKTHEADER";
      case ERRCODE_PKTNULL:
        return "ERRCODE_PKTNULL";
      case ERRCODE_PKTBUFOVERFLOW:
        return "ERRCODE_PKTBUFOVERFLOW";
      default:
        return "ERRCODE_SUCCESS";
    }
  }
};

}  // namespace lidar
}  // namespace timoo

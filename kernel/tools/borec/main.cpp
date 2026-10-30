// borec: records native results of unit 6G's algorithms for the port's tests (see common.hxx).
//   borec <check|volume|connected|defeature> <capture file, or - for none> <output file>
#include "common.hxx"

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::fprintf(stderr, "borec <check|volume|connected|defeature> <capture file or -> <output file>\n");
    return 1;
  }
  std::vector<borec::CapturedCase> aCases;
  if (std::strcmp(argv[2], "-") != 0)
    aCases = borec::ReadCaptures(argv[2]);
  std::ofstream anOut(argv[3], std::ios::binary);
  int           aStatus = 1;
  if (std::strcmp(argv[1], "check") == 0)
    aStatus = RecordCheck(aCases, anOut);
  else if (std::strcmp(argv[1], "volume") == 0)
    aStatus = RecordVolume(aCases, anOut);
  else if (std::strcmp(argv[1], "connected") == 0)
    aStatus = RecordConnected(aCases, anOut);
  else if (std::strcmp(argv[1], "defeature") == 0)
    aStatus = RecordDefeature(aCases, anOut);
  else
    std::fprintf(stderr, "unknown area %s\n", argv[1]);
  return aStatus;
}

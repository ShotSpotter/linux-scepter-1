
struct mcp3001_chip_info {
  bool is_present;
};

struct mcp3001_platform_data {
  struct mcp3001_chip_info chip;
  int    nvals;
};

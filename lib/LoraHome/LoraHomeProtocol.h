
enum PackageType {
    request,
    response,
};

struct Package
{
  uint8_t from_id;
  uint8_t to_id;
  PackageType type_package; // 0x01 - request, 0x02 - response
  uint16_t temperature;
};

Package createPackage(uint8_t from_id, uint8_t to_id, PackageType type, uint16_t temperature) {
Package package;

package.from_id = from_id;
package.to_id = to_id;
package.type_package = type;
package.temperature = temperature;

return package;
}
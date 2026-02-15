#include "utils.h"
#include <stdio.h>
#include <string.h>

void test_sha1() {
  printf("Testing SHA1...\n");
  SHA1_CTX ctx;
  unsigned char hash[20];
  char *input = "abc";

  SHA1Init(&ctx);
  SHA1Update(&ctx, (unsigned char *)input, strlen(input));
  SHA1Final(hash, &ctx);

  // Expected: a9993e36 4706816a ba3e2571 7850c26c 9cd0d89d
  unsigned char expected[] = {0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81,
                              0x6a, 0xba, 0x3e, 0x25, 0x71, 0x78, 0x50,
                              0xc2, 0x6c, 0x9c, 0xd0, 0xd8, 0x9d};

  if (memcmp(hash, expected, 20) == 0) {
    printf("SHA1 PASS\n");
  } else {
    printf("SHA1 FAIL\n");
    for (int i = 0; i < 20; i++)
      printf("%02x ", hash[i]);
    printf("\n");
  }
}

void test_base64() {
  printf("Testing Base64...\n");
  // "dGhlIHJlaW4gaW4gc3BhaW4gZmFsbHMgbWFpbmx5IG9uIHRoZSBwbGFpbg=="
  // "the rein in spain falls mainly on the plain" ->
  // dGhlIHJlaW4gaW4gc3BhaW4gZmFsbHMgbWFpbmx5IG9uIHRoZSBwbGFpbg== Actually let's
  // use the SHA1 expected result from above: a999... ->
  // qZk+NkcGgWq6PiVxeFDCbJzQ2J0=

  unsigned char input[] = {0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81,
                           0x6a, 0xba, 0x3e, 0x25, 0x71, 0x78, 0x50,
                           0xc2, 0x6c, 0x9c, 0xd0, 0xd8, 0x9d};
  char output[64];
  base64_encode(input, 20, output);

  char *expected = "qZk+NkcGgWq6PiVxeFDCbJzQ2J0=";

  if (strcmp(output, expected) == 0) {
    printf("Base64 PASS\n");
  } else {
    printf("Base64 FAIL\n");
    printf("Got: %s\n", output);
    printf("Exp: %s\n", expected);
  }
}

int main() {
  test_sha1();
  test_base64();
  return 0;
}

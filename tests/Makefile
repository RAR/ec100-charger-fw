CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can \
  test_oem_adc test_oem_rectifier test_oem_session test_oem_controller test_oem_ui \
  test_oem_integration test_oem_modbus test_oem_modbus_map test_rs485_mux test_oem_pylon test_oem_modbus_source test_oem_eg4 test_oem_jk test_oem_jkpb
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean

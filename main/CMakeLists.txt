add_executable(main consts.c utils.c main.c)

# pull in common dependencies
target_link_libraries(main pico_stdlib hardware_adc hardware_pwm hardware_clocks hardware_timer hardware_irq)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)

# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.c" 
         "r305_fingerprint.c"   
         "rc522_rfid.c"
         "LCD_DISPLAY.c"
         "buzzer.c"     # list the source files of this component
    INCLUDE_DIRS  "."
    EMBED_TXTFILES "certs/servercert.pem" 
                    "certs/prvtkey.pem" 
                    "server_certs/ca_cert.pem" 
                    "server_certs/ca_ota.pem" 
                    "server_certs/sbc_cert.pem"
)
spiffs_create_partition_image(storage ../SERVER FLASH_IN_PROJECT)

#ifndef __UCI_H__
#define __UCI_H__

#define UCI_INVALID_INT -999999

#define UCI_USER_FILE "/storage/emulated/0/uci_user.cfg"
#define UCI_SYS_FILE "/storage/emulated/0/uci_sys.cfg"
#define UCI_USER_FILE_END "uci_user.cfg"
#define UCI_SYS_FILE_END "uci_sys.cfg"

extern bool is_uci_path(const char *file_name);
extern bool is_uci_file(const char *file_name);

extern void notify_uci_file_closed(const char *file_name);

/** accesing kernel settings from UCI property configuration
*/
extern int uci_get_user_property_int(const char* property, int default_value);
extern int uci_get_user_property_int_mm(const char* property, int default_value, int min, int max);
/** accesing kernel settings from UCI property configuration
*/
extern const char* uci_get_user_property_str(const char* property, const char* default_value);

/** accesing sys variables from UCI sys props
*/
extern int uci_get_sys_property_int(const char* property, int default_value);
extern int uci_get_sys_property_int_mm(const char* property, int default_value, int min, int max);
/** accesing sys variables from UCI sys props
*/
extern const char* uci_get_sys_property_str(const char* property, const char* default_value);

#endif /* __UCI_H__ */

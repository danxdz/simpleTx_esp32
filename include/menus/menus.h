

static uint8_t params_loaded;     // if not zero, number received so far for current device
static uint8_t next_param;   // parameter and chunk currently being read
static uint8_t next_chunk;

//setup menus
int selected = 0;
int subSelected = -1;
int entered = -1; //-2 idle // -1 main menu // 0 options/submenu
bool menu_loaded = false;


class Menu {

    uint8_t hidden;
	char *value;
	char *info;
    uint8_t timeout;
    uint8_t min_value;
    uint8_t count;
    uint8_t default_value;

	public:
        
        uint8_t id;
        char *name;
        uint8_t parent;
        uint8_t p_type;
        uint8_t status;
        uint8_t max_value;
        char *optionsMainMenu[50];

        static void ChangeParam(uint8_t param, uint8_t cmd);
        static void loadMainMenu(char *load);

        char * getMainMenuItem_StatusText()
		{  
            if (optionsMainMenu[status]) return optionsMainMenu[status];
            return 0;
		}	             
        //debug
        void displayInfo()
		{			
            if (name) {
                db_out.printf("%u:%s:%u:%u:%u:%u:%u\n",
                                id,name,parent,p_type,hidden,max_value,status);
                if (max_value) {
                    for (size_t i = 0; i <= max_value; i++)
                    {
                        db_out.printf("%s:",
                        optionsMainMenu[i]);
                    }
                }      
                 
                if (p_type == 9) db_out.printf(" :: OPT");  
                if (p_type == 11) db_out.printf(" :: MainMenuItem");
                if (p_type == 12) db_out.printf(" :: INFO");
                if (p_type == 13) db_out.printf(" :: CMD");  

                db_out.printf("\n"); 
            }
        }

        void divideValueParam (char *values) {
            char *start = (char *)values;
            uint8_t count = 0;
            for (char *p = (char *)values; *p; p++) {
                if (*p == ';') {
                    int len = (strlen(start)-strlen(p));
                    optionsMainMenu[count] = new char[len+1];
                    strlcpy(optionsMainMenu[count],start,len+1);
                
                    start = p+1;
                    count += 1;
                }
            }
            int len = strlen(start);
            optionsMainMenu[count] = new char[len+1];
            strlcpy(optionsMainMenu[count],start,len+1);
                   
            max_value = count;   
        }

		void getParams(char *buffer,int iid)
		{
            ///db_out.printf("get P: %i\n",iid);
		    id = iid;
            parent = *buffer++;
            //set main menu items
                p_type = *buffer & 0x7f;
                hidden = *buffer++ & 0x80;
                
                name = new char[strlen(buffer)+1];
                strlcpy(name, (const char *)buffer,strlen(buffer)+1);
                buffer += strlen(buffer) + 1;
                
                switch (p_type) {
                case 9: //text
                    value = new char[strlen(buffer)+1];
                    strlcpy(value, (const char *)buffer,strlen(buffer)+1);
                    buffer += strlen(buffer) + 1;

                    divideValueParam(value);
                    //db_out.printf("%s",value);
                    parse_bytes(UINT8, &buffer, (char *) &status);
                    parse_bytes(UINT8, &buffer, (char *) &min_value);
                    parse_bytes(UINT8, &buffer, (char *) &count);  // don't use incorrect parameter->max_value
                    parse_bytes(UINT8, &buffer, (char *) &default_value);
        
                    break;
                case 11: //folder
                    max_value = 0;
                    break;
                case 12: //info
                    max_value = 0;   

                    value = new char[strlen(buffer)+1];
                    strlcpy(value, (const char *)buffer,strlen(buffer)+1);
                    buffer += strlen(buffer) + 1;
                    //db_out.printf("%s",value);

                    break;
                case 13: //command
                    parse_bytes(UINT8, &buffer, (char *)  &status);
                    //parse_bytes(UINT8, &buffer, (char *) &timeout);
                    max_value = 0;   
                    
                    //*(uint8_t *)status = (uint8_t) (*buffer)[0];
                    //*buffer += 1;
                    timeout = (uint8_t)buffer[0];
                    *buffer += 1;

                    info = new char[20];
                    strlcpy(info, (const char *)buffer, 20);
      
                default:
                    break;
                }
		}
};

Menu menuItems[CRSF_MAX_PARAMS];

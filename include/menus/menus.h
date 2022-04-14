
class Menu {

	uint8_t id;
    uint8_t parent;
    uint8_t p_type;
    uint8_t hidden;
	char *name;
	char *value;
    char *optionsMainMenu[50];
    uint8_t max_value;
	public:
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
		    id = iid;
            parent = *buffer++;
            //set main menu items
            if (parent == 0) {
                p_type = *buffer & 0x7f;
                hidden = *buffer++ & 0x80;
                
                name = new char[strlen(buffer)+1];
                strlcpy(name, (const char *)buffer,strlen(buffer)+1);
                buffer += strlen(buffer) + 1;
                
                value = new char[strlen(buffer)+1];
                strlcpy(value, (const char *)buffer,strlen(buffer)+1);
                buffer += strlen(buffer) + 1;

                divideValueParam(value);
                //db_out.printf("%s",value);
            } else {
                
            }


		}
		void getMarks()
		{
			
		}
		void displayInfo()
		{			
            if (name) {

                db_out.printf("\n %u:%s:%u:%u:%u:%u\n",
                                id,name,parent,p_type,hidden,max_value);
                for (size_t i = 0; i <= max_value; i++)
                {
                    db_out.printf("%s:",
                        optionsMainMenu[i]);
                }
                db_out.printf("\n");
            }
        }
};

Menu menuItems[CRSF_MAX_PARAMS];

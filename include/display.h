// Display queue system
typedef struct {
    char* text;
    int page;
} displayQueue_t;

// Initalize the display driver
void display_init();
void display_text(const char* text);
void display_write_page(const char* text, int page, bool isCenter = false);
void display_clear();
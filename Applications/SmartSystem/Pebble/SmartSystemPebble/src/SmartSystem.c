#include <pebble.h>


enum {
  KEY_BUTTON_EVENT = 0,
  KEY_BUTTON_UP = 1
};


Window* window;
MenuLayer *menu_layer;

//SEND MESSAGE FUNCTIONS

void send_int(uint8_t key, uint8_t cmd)
{
    DictionaryIterator *iter;
    app_message_outbox_begin(&iter);
      
    Tuplet value = TupletInteger(key, cmd);
    dict_write_tuplet(iter, &value);
      
    app_message_outbox_send();
}

static void in_received_handler(DictionaryIterator *iter, void *context) 
{
       
}


//MENU LAYER FUNCTIONS

void draw_row_callback(GContext *ctx, Layer *cell_layer, MenuIndex *cell_index, void *callback_context)
{
     //Which row is it?
    switch(cell_index->row)
    {
    case 0:
        menu_cell_basic_draw(ctx, cell_layer, "1. Visitor", "Alert level: high", NULL);
        break;
    case 1:
        menu_cell_basic_draw(ctx, cell_layer, "2. Operator", "Systems normal", NULL);
        break;
    case 2:
        menu_cell_basic_draw(ctx, cell_layer, "3. Worker", "Alert level: medium", NULL);
        break;
    case 3:
        menu_cell_basic_draw(ctx, cell_layer, "4. Maintenance", "Alert level: medium", NULL);
        break;
    }
}
 
uint16_t num_rows_callback(MenuLayer *menu_layer, uint16_t section_index, void *callback_context)
{
 return 4;
}

void select_click_callback(MenuLayer *menu_layer, MenuIndex *cell_index, void *callback_context)
{
    int row_num = cell_index->row;
  
    send_int(KEY_BUTTON_EVENT, row_num);
}


void window_load(Window *window)
{
     //Create it - 12 is approx height of the top bar
    menu_layer = menu_layer_create(GRect(0, 0, 144, 168 - 16));
 
    //Let it receive clicks
    menu_layer_set_click_config_onto_window(menu_layer, window);
 
    //Give it its callbacks
    MenuLayerCallbacks callbacks = {
        .draw_row = (MenuLayerDrawRowCallback) draw_row_callback,
        .get_num_rows = (MenuLayerGetNumberOfRowsInSectionsCallback) num_rows_callback,
        .select_click = (MenuLayerSelectCallback) select_click_callback
    };
    menu_layer_set_callbacks(menu_layer, NULL, callbacks);
 
    //Add to Window
    layer_add_child(window_get_root_layer(window), menu_layer_get_layer(menu_layer));
}
 
void window_unload(Window *window)
{
 menu_layer_destroy(menu_layer);
}
 
void init()
{
    window = window_create();
    WindowHandlers handlers = {
        .load = window_load,
        .unload = window_unload
    };
    window_set_window_handlers(window, (WindowHandlers) handlers);

    //Register AppMessage events
    app_message_register_inbox_received(in_received_handler);           
    app_message_open(512, 512);
    
    window_stack_push(window, true);
}
 
void deinit()
{
    window_destroy(window);
}
 
int main(void)
{
    init();
    app_event_loop();
    deinit();
}

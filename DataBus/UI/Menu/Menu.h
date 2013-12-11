#define _ITEM_MAX 10
#define NAMESIZ 20

typedef int (*FunctionPtr)();

/** Simple menu interface model
 */
class Menu {
public:

    /** Create a new menu model
     */
    Menu();
    
    /** add a new menu item
     */
    void add(const char *name, FunctionPtr f);
    
    /** select the next menu item as the current item
     */
    void next(void);
    
    /** select the previous menu item as the current item
     */
    void prev(void);
    
    /** run the function associated with the current item
     */
    void select(void);
    
    /** return the text for the current item
     */
    char *getItemName(void);

    /** return text for a specified item
     */
    char *getItemName(int i);
    
    /** print all the menu items
     */
    void printAll(void);
    
private:
    short _item;
    short _itemCount;
    char _name[_ITEM_MAX][NAMESIZ];
    FunctionPtr _exec[_ITEM_MAX];
};


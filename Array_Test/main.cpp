#include <iostream>

using namespace std;

int main()
{
    int test[] = {1,2,3};

    cout << sizeof(test)/sizeof(test[0]) << endl;
    return 0;
}

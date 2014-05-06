/**
 * @author 
 * @version 2014/05/06
 */

#include <iostream> 
using namespace std; 

int main() { 
    bool numImportant = false; 
    char input; 
    int rule1 = 0; 
    int rule2 = 0; 
    int strength = 0; 

    cout << "Are numeric digits in the password" << endl; 
    cout << "more important than alphabetic characters? "; 
    cin >> input; 
    if (input == 'Y') 
        numImportant = true; 

    cout << "Please enter the password in alphanumeric characters only"; 
    cout << " and terminated it by \"!\": "; 

    cin >> input; 

    while (input != '!') { 
        // One point for each alphanumeric character: 
        strength += 1; 
        if ((input >= 'A' && input <= 'Z') || (input >= 'a' && input <= 'z')) 
            // 10 points for at least one alphabetic character 
            rule1 = 10; 
        if (input >= '0' && input <= '9') { 
            // 10 points for at least one numeric digit: 
            rule2 = 10; 
            if (numImportant == true) 
                // One extra point for each numeric digit: 
                strength += 1; 
        } 
        cout << input; 
        cin >> input; 
    } 
    cout << endl; 

    strength += rule1 + rule2; 

    if (strength >= 20) 
        cout << "Good: "; 
    cout << "The strength of your password is " << strength << "." << endl; 

    return 0; 
}




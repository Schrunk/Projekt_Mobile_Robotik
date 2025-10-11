#ifndef LIST_HPP
#define LIST_HPP

#include <iostream>

/**
 * Template of a Stack List (LIFO - Last In, First Out)
 * Implementation of the most important list operations
 * 
 * Template T: Data type of the stored elements
 */
template<typename T>
class List {
private:
    // node structure of the linked list
    struct Node {
        T data;          // saved data
        Node* next;      // pointer to the next node

        // constructor for Node
        Node(const T& value) : data(value), next(nullptr) {}
    };

    Node* head;          // pointer to the first node (top of the stack)
    size_t size;         // number of elements in the list

public:
    // Constructor - creates an empty list
    List();
    
    // Destructor - frees all memory
    ~List();

    // Copy-Constructor
    List(const List& other);
    
    // Assignment-Operator
    List& operator=(const List& other);
    
    // add a element to the top of the list (stack: push)
    // @param value the element to add
    void push(const T& value);

    // removes the top element from the list (stack: pop)
    // @throws std::runtime_error if the list is empty
    void pop();

    // returns a reference to the top element (stack: top)
    // @return reference to the top element
    // @throws std::runtime_error if the list is empty
    T& top();

    // returns a const reference to the top element
    // @return const reference to the top element
    // @throws std::runtime_error if the list is empty
    const T& top() const;

    // returns the number of elements in the list
    // @return number of elements
    size_t length() const;

    // checks if the list is empty
    // @return true if empty, false otherwise
    bool isEmpty() const;

    // prints all elements of the list (from top to bottom)
    // @param separator separator between elements (default: " ")
    void print(const std::string& separator = " ") const;

    // clears the entire list
    void clear();

private:
    // Helper function to copy another list
    // @param other The list to copy
    void copyFrom(const List& other);
};

#include "list.tpp"

#endif // LIST_HPP

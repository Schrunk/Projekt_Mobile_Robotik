#ifndef LIST_CPP
#define LIST_CPP

#include "list.hpp"
#include <stdexcept>
#include <iostream>

// construktor
template<typename T>
List<T>::List() : head(nullptr), size(0) {
}

// destruktor
template<typename T>
List<T>::~List() {
    clear();
}

// copy-construktor
template<typename T>
List<T>::List(const List& other) : head(nullptr), size(0) {
    copyFrom(other);
}

// assignment-operator
template<typename T>
List<T>& List<T>::operator=(const List& other) {
    if (this != &other) {
        clear();
        copyFrom(other);
    }
    return *this;
}

// push - adds an element to the top of the list
template<typename T>
void List<T>::push(const T& value) {
    Node* newNode = new Node(value);
    newNode->next = head;
    head = newNode;
    size++;
}

// pop - removes the top element
template<typename T>
void List<T>::pop() {
    if (isEmpty()) {
        throw std::runtime_error("List is empty - pop() not possible");
    }
    
    Node* temp = head;
    head = head->next;
    delete temp;
    size--;
}

// top - returns a reference to the top element
template<typename T>
T& List<T>::top() {
    if (isEmpty()) {
        throw std::runtime_error("List is empty - top() not possible");
    }
    return head->data;
}

// top (const Version)
template<typename T>
const T& List<T>::top() const {
    if (isEmpty()) {
        throw std::runtime_error("List is empty - top() not possible");
    }
    return head->data;
}

// length - returns the number of elements
template<typename T>
size_t List<T>::length() const {
    return size;
}

// isEmpty - checks if the list is empty
template<typename T>
bool List<T>::isEmpty() const {
    return size == 0;
}

// print - prints all elements
template<typename T>
void List<T>::print(const std::string& separator) const {
    if (isEmpty()) {
        std::cout << "List is empty" << std::endl;
        return;
    }

    std::cout << "List (Top -> Bottom): ";
    Node* current = head;
    bool first = true;
    
    while (current != nullptr) {
        if (!first) {
            std::cout << separator;
        }
        std::cout << current->data;
        current = current->next;
        first = false;
    }
    std::cout << std::endl;
}

// clear - clears the entire list
template<typename T>
void List<T>::clear() {
    while (!isEmpty()) {
        pop();
    }
}

// helper function to copy
template<typename T>
void List<T>::copyFrom(const List& other) {
    if (other.isEmpty()) {
        return;
    }

    // temporary stack to reverse the order
    Node* temp_stack = nullptr;
    Node* current = other.head;

    // collect all nodes in reverse order into temp_stack
    while (current != nullptr) {
        Node* newNode = new Node(current->data);
        newNode->next = temp_stack;
        temp_stack = newNode;
        current = current->next;
    }

    // push from temp_stack in the correct order
    while (temp_stack != nullptr) {
        Node* temp = temp_stack;
        push(temp->data);
        temp_stack = temp_stack->next;
        delete temp;
    }
}

#endif // LIST_CPP

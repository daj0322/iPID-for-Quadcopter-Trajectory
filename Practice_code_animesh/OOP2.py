class Person:
    def __init__ (self,name,age):
        self.name = name
        self.age = age
    
    def greet(self):
        print(f"Hello my name is {self.name} and I am {self.age} years old")

Person1 = Person('Animesh',20)

print(Person1.greet())

##
class User:
    def __init__(self,username,email,password):
        self.username = username
        self._email = email
        self.password = password
    
    def get_email(self):
        return self._email
    
    def set_email(self,email):
        self._email = email
        
    
user1 = User("sadasd","animesh.s@gmail.com","asdsad")
print(user1.get_email())

user1.set_email("abanga4@lsu.edu")
print(user1.get_email())


##
#%%
class  User:
    def __init__(self,username,email,password):
        self.username = username
        self._email = email
        self.__password = password
        
    @property
    def email(self):
        print("Email accessed")
        return self._email
    
    @email.setter
    def email(self,new_email):
        if "@" in new_email:
            self._email = new_email
        
    
        
user1 = User("dantheman","dan@gmail.com","123")
user1.email = "this is not an email"
print(user1.email)        

# %%
# Statice attributes
# A static attribute (sometimes called a class attribute) is an attribute that belongs to the class itself.
#not to any specific instance of the class.















# %%

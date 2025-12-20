class User:
    def __init__(self,username,email,password):
        self.username = username
        self.email = email
        self.password = password
    def say_hi_to_user(self,user):
        print(f"Message from {self.username} : Hello {user.username}, myself {self.username}")
user1 = User("adsad","adsd@","asdasd")
print(user1.email)

user1.email = "dan"

print(user1.email)



## The "Consenting Adults" Philosophy
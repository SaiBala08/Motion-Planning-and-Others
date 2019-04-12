



from IPython.display import clear_output

def display_board(board):
    
    clear_output()
    
    print(test_board[7]+'|'+test_board[8]+'|'+test_board[9])
    print(test_board[4]+'|'+test_board[5]+'|'+test_board[6])
    print(test_board[1]+'|'+test_board[2]+'|'+test_board[3])



def player_input():
    
    P1_marker = ''
    P2_marker = ''
    
    while P1_marker not in ('x','o'):
        P1_marker = input("Please pick a marker 'X' or 'O'.").lower()
        
    if P1_marker == 'x':
        P2_marker = 'o'
        return ('X','O')
    else:
        P2_marker = 'x'
        return ('O','X')
        
    print(f"player1 has choose {P1_marker} and so player2 is {P2_marker}")
    print('You are now ready to play')
    



def place_marker(board,marker,position):
    
    board[position] = marker





def win_check(board, mark):
    
    if test_board[1] == test_board[2] ==  test_board[3] == mark:
        return mark    
    elif test_board[4] == test_board[5] ==  test_board[6] == mark:
        return mark 
    elif test_board[7] ==  test_board[8] ==  test_board[9] == mark:
        return mark 
    elif test_board[1] ==  test_board[4] ==  test_board[7] == mark:
        return mark 
    elif test_board[2] ==  test_board[5] ==  test_board[8] == mark:
        return mark 
    elif test_board[3] ==  test_board[6] ==  test_board[9] == mark:
        return mark 
    elif test_board[1] ==  test_board[5] ==  test_board[9] == mark:
        return mark 
    elif test_board[7] ==  test_board[5] ==  test_board[3] == mark:
        return mark 
    else:
        return ''
    



win_check(test_board,'x') 


#


import random

def choose_first():
    return random.choice(['player1','player2'])




def space_check(board,position):
    
    if board[position] != ' ':
        print("The position is already occupied")
        return False
    else:
        return True




def full_board_check(board):
    
    b = 0
    
    for x in board[1:]:
        if x == ' ':
            b += 1
        
    if b == 0:
        return True
    else:
        return False





def player_choice(board):
    
    position = 0
    
    while position not in range(1,10):
        position = int(input('please provide your move from 1-9:'))
    
        if space_check(board,position):
            return position
        else:
            print("Try another move")
            position = 0
            continue




def replay():
    
    replay_input = input("Do you want to play again. Say yes or no: ")
    
    while replay_input == 'yes' or replay_input == 'no':
        if replay_input == 'yes':
            return True
        elif replay_input == 'no':
            return False


# In[ ]:


print('Welcome to Tic Tac Toe!')

while True:
    print('player1 choose your marker')
    [P1_marker, P2_marker] = player_input()
    
    test_board = [' ']*10
    
    first_player = choose_first()
    print(f'{first_player} will go first')  
    
    a = 0
    while win_check(test_board,'x') != 'x' or win_check(test_board,'o') != 'o':
        
        if first_player == 'player1' and a == 0:
            a += 1
            print('Player1 choose your move')
            position = player_choice(test_board)
            place_marker(test_board,P1_marker,position)
            display_board(test_board)
            
        elif first_player == 'player2' and a == 0:
            a += 2
            print('Player2 choose your move')
            position = player_choice(test_board)
            place_marker(test_board,P2_marker,position)
            display_board(test_board)
            
            
        if a%2 != 0:
            a += 1
            if win_check(test_board,P1_marker) == P1_marker or win_check(test_board,P2_marker) == P2_marker:
                break
            print('Player2 choose your move')
            position = player_choice(test_board)
            place_marker(test_board,P2_marker,position)
            display_board(test_board)
            
        else:
            a += 1
            if win_check(test_board,P1_marker) == P1_marker or win_check(test_board,P2_marker) == P2_marker:
                break
            print('Player1 choose your move')
            position = player_choice(test_board)
            place_marker(test_board,P1_marker,position)
            display_board(test_board)
           
            
        
        if a == 9 or a == 10:
            if full_board_check(test_board):
                print('Game Over, the board is filled and no player has won')
                break
            
    
    if win_check(test_board,'X') == 'X':
        if P1_marker == 'X':
            print('Congragulations!, Player1 has won the game')
        else:
            print('Congragulations!, Player2 has won the game')
            
    if win_check(test_board,'O') == 'O':
        if P1_marker == 'O':
            print('Congragulations!, Player1 has won the game')
        else:
            print('Congragulations!, Player2 has won the game')
    
    if replay():
        continue
    else:
        print('Good Bye')
    
    break
        
        

挙動のチェックリスト
1.start -> next_rq exitst -> not exceed batch_read -> not continously dispatch 8 times in same layer -> DISPATCH!
修正
2.start -> next_rq exitst -> not exceed batch_read -> continously dispatch 8 times in same layer -> select next layer -> no req -> 




* TPGS I/O Scheduler Reading Memo
** data structure
-struct kmem_cache *tpgs_pool

-struct tpgs_queue
オンラインかどうか
リクエストを保持しているかどうか
などを保持している

-struct tppg_stats
統計情報を保持

-struct tpgs_group
デバイス毎のcgroupsの構造


-struct tpgs_io_cq
I/Oコンテキストとキューをとりもつ

-struct tpgs_data
メインデータを保持している
リクエストキューだったりを保持

** helper functions
** Elevator API's
*** elevator_merged_fn = tpgs_merged_request
スケジューラ内のリクエストがマージされた時に呼ばれる

*** elevator_merge_req_fn = tpgs_merged_requests
２つのリクエストがマージされると呼ばれる

*** elevator_dispatch_fn = tpgs_dispatch_requests
ディスパッチする際に呼ばれる。
drainによってNULLになるまでwhileで呼ばれ続ける

*** elevator_add_req_fn = tpgs_insert_request
スケジューラにリクエストを追加する際によばれる

*** elevator_activate_req_fn tpgs_activate_request
デバドラがリクエストを最初に見るときに呼ばれる
tppd->rq_in_driver及びtppg->rq_in_driverのインクリメント

*** elevator_deactivate_req_fn tpgs_deactivate_request
デバドラがリクエストをリキューすることによって遅らせることを決定した際によばれる
tppd->rq_in_driver tppg->rq_in_driverのデクリメント

*** elevator_completed_req_fn tpgs_completed_request
リクエストが完了した際に呼ばれる
rq_in_driverのデクリメント
tppg_stats_update_complettion()でtppgのアプデ
rq_in_driverが０ならtpgs_scheduler_dispatch()

*** elevator_init_icq_fn
io_cqを作成してリンクする際(ioc_create_icq())に呼ばれる

*** elevator_exit_icq_fn
io_cqを破棄する際に呼ばれる
*** elevator_set_req_fn tpgs_set_request
リクエストの為にスケジューラ内のデータを確保する際に呼ばれる

*** elevator_put_req_fn
リクエストの為にスケジューラ内のデータを開放する際に呼ばれる

*** elevator_init_fn = tpgs_init_queue
scheduler固有データのmalloc、初期化
blkcg_activate_policy()によってblkcg_policy_tpgsをアクティベート
ルートグループ(tppd->root_group)の初期化
INIT_WORK

*** elevator_exit_fn = tpgs_exit_queue
cancel_work_syncによるアンプラギングの解除
blkcg_deactivate_policyによるポリシーの解除
確保したメモリの開放




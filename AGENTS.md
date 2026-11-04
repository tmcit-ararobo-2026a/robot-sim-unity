# Unity エージェント操作

このリポジトリは Unity 6000.0.85f1 の ROS 2 ロボットシミュレータです。
Unity Editor の操作には `com.unity.pipeline` と Unity CLI を使います。
作業前に `.agents/skills/unity-pipeline/SKILL.md` と、そこから参照される
導入済みパッケージの `Documentation~/` の該当ページを読んでください。
操作方法と復旧手順は `docs/unity-agent.md` にあります。

- 最初に `git status --short` で既存の変更を確認し、ユーザーの変更を保持します。
- Editor を操作する呼び出しには、このチェックアウトの絶対パスを
  `--project-path` で指定します。別のチェックアウトの Editor は使いません。
- シーン・Prefab・Material は起動中の Editor 経由で変更し、保存後に差分を確認します。
  `.unity` / `.prefab` の YAML を直接書き換える前に接続状態を確認してください。
- C# は通常のファイル編集後に `unity recompile` でコンパイル結果まで確認します。
- 一行の確認には `script eval`、繰り返し使う処理には `Assets/` 外の C# ファイルと
  `script run` を使います。コマンド名・引数は接続先の一覧で確認してください。
- 検証後は `console status` と必要に応じて `console get --level error` を確認します。
  Play Mode で検証する場合はフレーム・シミュレーション時刻の進行も確認します。
- ROS 2 通信、機体の物理値、センサーの取得時刻・座標系・配信形式を変更した場合は、
  関連する `Assets/Scripts/Editor/*Verification.cs` の既存検証を実施します。
- `Library/`、`Temp/`、`Logs/`、`UserSettings/` は生成物です。コミットしません。
- 既存の uLoopMCP は併存しています。削除や設定変更は依頼の範囲でのみ行います。

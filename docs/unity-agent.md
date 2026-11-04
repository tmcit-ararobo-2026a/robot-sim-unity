# Unity Pipeline でのエージェント操作

このプロジェクトには公式 Unity Pipeline (`com.unity.pipeline`、`0.10.0-exp.1`)
を導入しています。Unity 6000.0.85f1 と Unity CLI `1.0.0-beta.16` で接続を確認しています。
Pipeline は実験版パッケージです。Editor を起動しておくと、エージェントからシーンの取得・編集、
C# 実行、再コンパイル、Play Mode 操作、コンソール取得ができます。

## 初回セットアップ

1. Unity CLI がなければ [公式インストーラ](https://unity.com/install.sh) で導入します。
2. このリポジトリのルートで以下を実行します。パッケージは manifest と lock に記録済みなので、
   通常のクローンでは Unity Package Manager が自動取得します。

```bash
unity --version
unity open "$PWD"
unity status --until-ready --timeout 60 --project-path "$PWD" --format json
unity command editor status --project-path "$PWD" --format json
```

Codex 向けの公式 `unity-pipeline` スキルは `.agents/skills/unity-pipeline/` に保存済みです。
シェルを使えるエージェントは CLI 経由で操作できます。追加の MCP サーバー登録は不要です。

## 普段の操作

以下はすべてリポジトリのルートから実行します。別ディレクトリから使う場合は
`--project-path` にこのチェックアウトの絶対パスを指定します。

```bash
# 公開されているコマンドと引数を確認
unity command --project-path "$PWD" --format json
unity command scene --project-path "$PWD" --format json
unity command --query recompile --detail full --project-path "$PWD" --format json

# C# を実行して現在のシーンを取得
unity command script eval 'return UnityEngine.SceneManagement.SceneManager.GetActiveScene().path;' --project-path "$PWD" --format json

# C# 編集後の再コンパイル（完了まで待って結果を返す）
unity recompile --project-path "$PWD" --timeout 120 --format json

# Play Mode の開始と停止
unity command editor.playmode start --project-path "$PWD" --format json
unity status --until-ready --timeout 60 --project-path "$PWD" --format json
unity command editor.playmode stop --project-path "$PWD" --format json

# コンソールを消さずにエラーを取得
unity command console status --project-path "$PWD" --format json
unity command console get --level error --tail 30 --project-path "$PWD" --format json
```

背面の Editor でコンパイルやテストを進める場合は、先に
`unity command editor set_auto_tick --enabled true --project-path "$PWD"` を実行します。
検証後は元の設定に戻し、変更した実行状態を作業報告に記録してください。
`--persist false` を付けると、次のドメインリロードで保存済みの設定に戻ります。
Play Mode に入れたことだけで実行成功とせず、フレーム・時刻の進行とログを確認します。

既存のセンサー検証も一行の C# 呼び出しで実行できます。例は Edit Mode 用の配信検証です。

```bash
unity command script eval 'return RobotSim.Editor.Mid360DeliveryVerification.Run();' --project-path "$PWD" --format json
```

CLI の正常終了とは別に、返された検証レポートの `passed` を確認してください。
この検証は `Temp/Mid360DeliveryVerification.json` を出力します。
ほかの検証は `Assets/Scripts/Editor/*Verification.cs` を読み、必要なシーン・Play Mode 条件を確認して実施します。

## 接続できないとき

- `unity status --project-path "$PWD" --format json` と
  `unity pipeline editors --project-path "$PWD" --format json` で状態を確認します。
- インストール直後に `STATUS_PIPELINE_LOAD_PENDING` なら Editor のインポート完了を待ちます。
  完了しても続く場合は Unity ウィンドウにフォーカスして refresh させます。
- Safe Mode なら表示されたコンパイルエラーを修正します。必要なログだけを
  `Logs/Editor.log`、なければ Linux の `~/.config/unity3d/Editor.log` から抽出します。
- サンドボックス内だけで Editor が見えない場合は、ローカルプロセスの検出や localhost 接続が
  制限されている可能性があります。承認された実行経路で該当する CLI コマンドだけを実行します。
- `editor status` がモーダルダイアログを報告した場合は、ユーザーに内容を伝えて解消を待ちます。

パッケージ更新は `unity pipeline install --force --project-path "$PWD"` で行い、
manifest と lock の差分、再コンパイル、関連する検証を確認します。
プロジェクト内スキルの更新は
`unity skill install codex --local --skill unity-pipeline --yes` で行えます。
インストールしたバージョンに一致するコマンド仕様は
`Library/PackageCache/com.unity.pipeline@*/Documentation~/` を優先してください。

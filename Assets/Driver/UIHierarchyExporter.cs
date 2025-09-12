using UnityEngine;
using UnityEditor;
using UnityEngine.SceneManagement;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using System.Collections.Generic;


public class UIHierarchyExporter
{
    [MenuItem("Tools/Export UI Hierarchy")]
    public static void ExportUIHierarchy()
    {
        // 输出路径（项目根目录下生成 UIHierarchy.txt）
        string sceneName = SceneManager.GetActiveScene().name;
        string fileName = $"UIHierarchy_{sceneName}.txt";
        string path = Path.Combine(Application.dataPath, "../" + fileName);

        StringBuilder sb = new StringBuilder();
        sb.AppendLine("=== UI Hierarchy Export ===");
        sb.AppendLine("Scene: " + SceneManager.GetActiveScene().name);
        sb.AppendLine();

        // 遍历场景根节点
        GameObject[] rootObjects = SceneManager.GetActiveScene().GetRootGameObjects();
        foreach (GameObject root in rootObjects)
        {
            TraverseHierarchy(root.transform, sb, 0);
        }

        File.WriteAllText(path, sb.ToString(), Encoding.UTF8);
        Debug.Log("UI Hierarchy exported to: " + path);
        EditorUtility.RevealInFinder(path);
    }
    

    private static void TraverseHierarchy(Transform trans, StringBuilder sb, int depth)
    {
        string indent = new string(' ', depth * 2);

        // 判断是否需要折叠
        string lowerName = trans.name.ToLower();
        bool isIgnoredNode = lowerName.Contains("lod") ||
                            lowerName.Contains("mesh") ||
                            lowerName.Contains("attachment") ||
                            lowerName.Contains("rootnode") ||
                            Regex.IsMatch(lowerName, @"\{[0-9a-f\-]{36}\}");

        bool hasScripts = trans.GetComponents<MonoBehaviour>().Length > 0;

        // 输出节点名
        sb.Append(indent).Append("- ").Append(trans.name);
        if (hasScripts)
        {
            sb.Append(" [Scripts: ");
            MonoBehaviour[] scripts = trans.GetComponents<MonoBehaviour>();
            for (int i = 0; i < scripts.Length; i++)
            {
                sb.Append(scripts[i]?.GetType().Name ?? "MissingScript");
                if (i < scripts.Length - 1) sb.Append(", ");
            }
            sb.Append("]");
        }
        else if (isIgnoredNode)
        {
            sb.Append(" [Ignored]");
        }
        sb.AppendLine();

        // 收集子节点
        List<Transform> children = new List<Transform>();
        foreach (Transform child in trans) children.Add(child);

        // 扫描子节点，折叠连续编号
        for (int i = 0; i < children.Count; i++)
        {
            string name = children[i].name;
            Match m = Regex.Match(name, @"^(.*?)(\d+)$"); // prefix + number
            if (m.Success)
            {
                string prefix = m.Groups[1].Value;
                int startNum = int.Parse(m.Groups[2].Value);
                int endNum = startNum;

                int j = i + 1;
                while (j < children.Count)
                {
                    Match mj = Regex.Match(children[j].name, @"^(.*?)(\d+)$");
                    if (mj.Success && mj.Groups[1].Value == prefix)
                    {
                        int num = int.Parse(mj.Groups[2].Value);
                        if (num == endNum + 1)
                        {
                            endNum = num;
                            j++;
                            continue;
                        }
                    }
                    break;
                }

                // 序列超过 3 个才折叠
                if (endNum > startNum + 2)
                {
                    sb.Append(indent).Append("  - ").Append(prefix)
                    .Append(startNum.ToString("D3")).Append(" ... ").Append(prefix)
                    .Append(endNum.ToString("D3"));

                    if (isIgnoredNode) sb.Append(" [Ignored]");
                    else sb.Append(" [Scripts: WayPoint]"); // 根据需要调整

                    sb.AppendLine();
                    i = j - 1;
                    continue;
                }
            }

            // 普通节点递归
            TraverseHierarchy(children[i], sb, depth + 1);
        }
    }



}

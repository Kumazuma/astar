#include "prec.h"

#include <wx/wx.h>

class AStarApp : public wxApp
{
public:
    bool OnInit() override;

private:
    wxFrame* m_pAStarFrame;
};

class AStarFrameUi : public wxFrame
{
public:
    AStarFrameUi(wxWindow* win, wxWindowID id, const wxString& title)
     : wxFrame(win, id, title) {
        
    }
};


namespace astar
{
    struct Node;
    struct Link
    {
        float depth;
        Node* a;
        Node* b;
    };

    struct Node
    {
        float x;
        float y;
    };

    class Map
    {
    public:
        struct NavigatedNode
        {
            Node* node;
            NavigatedNode* prevNode;
            float g;
            float h;
            bool operator == (Node* rhs) const
            {
                return node == rhs;
            }

            operator Node* () const
            {
                return node;
            }
        };

        template<typename T>
        struct Ref
        {
            Ref(T& t) : ref{ t } {}
            
            template<typename RHS>
            bool operator == (const RHS& rhs) const
            {
                return ref == rhs;
            }

            bool operator == (const Ref& rhs) const
            {
                return ref == rhs.ref;
            }

            T* operator->() { return &ref; }

            operator T& () const
            {
                return ref;
            }

            T& ref;
        };

        ~Map()
        {
            for (auto it : m_nodeList)
            {
                delete it;
            }

            for (auto it : m_linkList)
            {
                delete it;
            }
        }

        Node* AddNode(float x, float y)
        {
            auto ptr = new Node{};
            ptr->x = x;
            ptr->y = y;
            m_nodeList.push_back(ptr);
            return ptr;
        }

        void UpdateLength()
        {
            for (auto it : m_linkList)
            {
                it->depth = Length(it->a, it->b);
            }
        }

        Link* MakeLink(Node* node1, Node* node2)
        {
            for (auto it : m_linkList)
            {
                if (it->a != node1 && it->b != node1)
                    continue;

                if (it->a != node2 && it->b != node2)
                    continue;

                return nullptr;
            }

            auto link = new Link{};
            link->a = node1;
            link->b = node2;
            link->depth = Length(node1, node2);

            m_linkList.push_back(link);
            return link;
        }

        void RemoveLink(Link* link)
        {

        }
        
        void RemoveNode(Node* node)
        {

        }

        bool Navigate(Node* from, Node* to, std::vector<Node*>* result)
        {
            if (from == to)
                return true;

            std::vector<NavigatedNode> nodeList;
            std::list<Ref<NavigatedNode>> navigatedNodeList;

            for (auto it : m_nodeList)
            {
                NavigatedNode node{};
                node.node = it;
                node.h = Length(it, to);
                node.g = std::numeric_limits<float>::max();
                node.prevNode = nullptr;
                nodeList.push_back(node);
            }

            Ref<NavigatedNode> navNodeFrom = *std::find(nodeList.begin(), nodeList.end(), from);
            Ref<NavigatedNode> navNodeTo = *std::find(nodeList.begin(), nodeList.end(), to);
            navNodeFrom->g = 0.f;
            navigatedNodeList.push_back(navNodeFrom);
            do {
                if (navigatedNodeList.empty())
                    return false;

                auto sel = navigatedNodeList.front();
                navigatedNodeList.pop_front();
                if (sel == navNodeTo)
                    break;

                auto nearNodeList = CollectLinkedNode(sel, nodeList);
                for (auto node : nearNodeList)
                {
                    if (node->prevNode == &sel.ref)
                        continue;

                    float g = Length(node->node, sel->node);
                    if (node->g <= g + sel->g)
                        continue;

                    node->g = g + sel->g;
                    node->prevNode = &sel.ref;
                    auto it = std::find(navigatedNodeList.begin(), navigatedNodeList.end(), node);
                    if (it != navigatedNodeList.end())
                    {
                        // 기존 아이템 삭제
                        it = navigatedNodeList.erase(it);
                    }

                    auto end = navigatedNodeList.end();
                    it = navigatedNodeList.begin();
                    
                    for (; it != end; ++it)
                    {
                        auto& rhs = *it;
                        if (node->g + node->h >= rhs->g + rhs->h)
                            continue;

                        navigatedNodeList.insert(it, node);
                        break;
                    }

                    if (it == end)
                    {
                        navigatedNodeList.push_back(node);
                    }
                }
            } while (true);
            
            result->clear();
            {
                NavigatedNode* it = &navNodeTo.ref;
                while (it != nullptr)
                {
                    result->push_back(it->node);
                    it = it->prevNode;
                }

                std::reverse(result->begin(), result->end());
            }

            return true;
        }

    private:
        static float Length(Node* node1, Node* node2)
        {
            float dx = node1->x - node2->x;
            float dy = node1->y - node2->y;
            return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        }

        std::vector<Ref<NavigatedNode>> CollectLinkedNode(Ref<NavigatedNode> node, std::vector<NavigatedNode>& navNode)
        {
            std::vector<Ref<NavigatedNode>> ret;
            for (auto it : m_linkList)
            {
                if (it->a == node.ref.node)
                {
                    ret.push_back(*std::find(navNode.begin(), navNode.end(), it->b));
                }
                else if (it->b == node.ref.node)
                {
                    ret.push_back(*std::find(navNode.begin(), navNode.end(), it->a));
                }
            }

            return ret;
        }

    private:
        std::vector<Node*> m_nodeList;
        std::vector<Link*> m_linkList;
    };
}

enum {
    ID_ADD_NODE = wxID_HIGHEST + 1,
    ID_ADD_LINK,
    ID_MOVE_NODE,
    ID_FIND_PATH
};

class AStarFrame : public AStarFrameUi
{
public:
    AStarFrame(wxWindow* win, wxWindowID id, const wxString& title)
        : AStarFrameUi(win, id, title)
        , m_selectedToolId{ ID_MOVE_NODE } {
        Bind(wxEVT_CONTEXT_MENU, [this](wxContextMenuEvent& evt)
            {
                wxMenu menu;
                menu.Append(ID_ADD_NODE, wxS("노드 추가"));
                menu.AppendRadioItem(ID_ADD_LINK, wxS("노드 연결"));
                menu.AppendRadioItem(ID_MOVE_NODE, wxS("노드 이동"));
                menu.AppendRadioItem(ID_FIND_PATH, wxS("길 찾기"));
                menu.Check(m_selectedToolId, true);
            PopupMenu(&menu);
        });

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
        {
            auto size = GetClientSize();
            size /= 2;
            wxPoint pt{ size.x, size.y };
            auto node = m_map.AddNode(pt.x, pt.y);
            m_nodeList.push_back(node);
            Refresh();
            m_selectedToolId = ID_MOVE_NODE;
        }, ID_ADD_NODE);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_ADD_LINK;
            }, ID_ADD_LINK);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_MOVE_NODE;
            }, ID_MOVE_NODE);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_FIND_PATH;
            }, ID_FIND_PATH);

        Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent& evt)
        {
            if (m_selectedToolId == ID_MOVE_NODE)
            {
                Refresh();
                m_basePosition = evt.GetPosition();
                m_selectedNode = GetHitNode(m_basePosition);
                if (m_selectedNode != nullptr)
                {
                    m_oldNodePosition.x = m_selectedNode->x;
                    m_oldNodePosition.y = m_selectedNode->y;
                }
            }
            else if (m_selectedToolId == ID_ADD_LINK)
            {
                Refresh();
                auto selectedNode = GetHitNode(evt.GetPosition());
                if (m_selectedNode == nullptr)
                {
                    m_selectedNode = selectedNode;
                    return;
                }
                
                if (selectedNode == nullptr)
                {
                    m_selectedNode = nullptr;
                    return;
                }

                auto link = m_map.MakeLink(selectedNode, m_selectedNode);
                m_selectedNode = nullptr;
                if (link != nullptr)
                {
                    m_linkList.push_back(link);
                }
            }
            else if(m_selectedToolId == ID_FIND_PATH)
            {
                Refresh();
                auto selectedNode = GetHitNode(evt.GetPosition());
                if (m_selectedNode == nullptr)
                {
                    m_pathNodeList.clear();
                    m_selectedNode = selectedNode;
                    return;
                }

                if (selectedNode == nullptr)
                {
                    m_pathNodeList.clear();
                    m_selectedNode = nullptr;
                    return;
                }

                m_map.UpdateLength();
                m_map.Navigate(m_selectedNode, selectedNode, &m_pathNodeList);
                m_selectedNode = nullptr;
            }
        });

        Bind(wxEVT_MOTION, [this](wxMouseEvent& evt)
        {
                if (m_selectedToolId != ID_MOVE_NODE)
                    return;
                
                if (m_selectedNode == nullptr)
                    return;

                Refresh();
                wxPoint newPos = m_oldNodePosition;
                newPos += evt.GetPosition() - m_basePosition;
                m_selectedNode->x = newPos.x;
                m_selectedNode->y = newPos.y;
        });

        Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& evt)
        {
            if (m_selectedToolId == ID_MOVE_NODE && m_selectedNode != nullptr)
            {
                m_selectedNode = nullptr;
                Refresh();
            }
        });

        Bind(wxEVT_PAINT, [this](wxPaintEvent& evt)
            {
                wxPaintDC dc{ this };
                wxBrush whiteBrush{ *wxWHITE_BRUSH };
                wxBrush blackBrush{ *wxBLACK_BRUSH };
                for (auto it : m_linkList)
                {
                    dc.DrawLine(it->a->x, it->a->y, it->b->x, it->b->y);
                }

                dc.SetBrush(whiteBrush);
                for (auto it : m_nodeList)
                {
                    if (it == m_selectedNode)
                    {
                        dc.SetBrush(blackBrush);
                    }

                    dc.DrawEllipse(wxPoint(it->x - 10, it->y - 10), wxSize{ 21, 21 });

                    if (it == m_selectedNode)
                    {
                        dc.SetBrush(whiteBrush);
                    }
                }

                dc.SetBrush(*wxRED_BRUSH);
                for (auto it : m_pathNodeList)
                {
                    dc.DrawEllipse(wxPoint(it->x - 10, it->y - 10), wxSize{ 21, 21 });
                }

                if (!m_pathNodeList.empty() && m_selectedNode != nullptr)
                {
                    dc.SetBrush(blackBrush);
                    dc.DrawEllipse(wxPoint(m_selectedNode->x - 10, m_selectedNode->y - 10), wxSize{ 21, 21 });
                }
            });
    }

    astar::Node* GetHitNode(const wxPoint& pt)
    {
        auto it = m_nodeList.rbegin();
        for (; it != m_nodeList.rend(); ++it)
        {
            auto node = *it;
            float dx = node->x - pt.x;
            float dy = node->y - pt.y;
            if (dx * dx + dy * dy < 441)
            {
                return node;
            }
        }

        return nullptr;
    }

private:
    astar::Map m_map;
    std::vector<astar::Node*> m_nodeList;
    std::vector<astar::Link*> m_linkList;
    std::vector<astar::Node*> m_pathNodeList;
    wxPoint m_basePosition;
    wxPoint m_oldNodePosition;
    astar::Node* m_selectedNode;
    wxWindowID m_selectedToolId;
};

bool AStarApp::OnInit()
{
    if (!wxApp::OnInit())
    {
        return false;
    }

    m_pAStarFrame = new AStarFrame{ nullptr, wxID_ANY, wxS("A* test") };
    m_pAStarFrame->Show();

    return true;
}

wxIMPLEMENT_APP(AStarApp);
